#include "hexapod_controller/ik.hpp"



Ik::Ik( const rclcpp::NodeOptions &options )
: rclcpp::Node ("ik",options)
{
    // Declare parameter in node
    this->declare_parameter( "COXA_TO_CENTER_X", std::vector<double>{} );
    this->declare_parameter( "COXA_TO_CENTER_Y", std::vector<double>{} );
    this->declare_parameter( "INIT_COXA_ANGLE", std::vector<double>{} );
    this->declare_parameter( "INIT_FOOT_POS_X", std::vector<double>{} );
    this->declare_parameter( "INIT_FOOT_POS_Y", std::vector<double>{} );
    this->declare_parameter( "INIT_FOOT_POS_Z", std::vector<double>{} );
    this->declare_parameter( "COXA_LENGTH",0.000 );
    this->declare_parameter( "FEMUR_LENGTH",0.000 );
    this->declare_parameter( "TIBIA_LENGTH",0.000 );
    this->declare_parameter( "TARSUS_LENGTH",0.000 );
    this->declare_parameter( "NUMBER_OF_LEGS",0 );

    // Initialize variables
    COXA_TO_CENTER_X = this->get_parameter( "COXA_TO_CENTER_X" ).as_double_array();
    COXA_TO_CENTER_Y = this->get_parameter( "COXA_TO_CENTER_Y" ).as_double_array();
    INIT_COXA_ANGLE = this->get_parameter( "INIT_COXA_ANGLE" ).as_double_array();
    INIT_FOOT_POS_X = this->get_parameter( "INIT_FOOT_POS_X" ).as_double_array();
    INIT_FOOT_POS_Y = this->get_parameter( "INIT_FOOT_POS_Y" ).as_double_array();
    INIT_FOOT_POS_Z = this->get_parameter( "INIT_FOOT_POS_Z" ).as_double_array();
    COXA_LENGTH = this->get_parameter( "COXA_LENGTH" ).as_double();
    FEMUR_LENGTH = this->get_parameter( "FEMUR_LENGTH" ).as_double();
    TIBIA_LENGTH = this->get_parameter( "TIBIA_LENGTH" ).as_double();
    TARSUS_LENGTH = this->get_parameter( "TARSUS_LENGTH" ).as_double();
    NUMBER_OF_LEGS = this->get_parameter( "NUMBER_OF_LEGS" ).as_int();
    

}

//=============================================================================
// getSinCos:  Get the sinus and cosinus from the angle
//=============================================================================

Trig Ik::getSinCos( double angle_rad )
{
    Trig body_trig;

    body_trig.sine = sin( angle_rad );
    body_trig.cosine = cos( angle_rad );

    return body_trig;
}

//=============================================================================
// Inverse Kinematics
//=============================================================================

void Ik::calculateIK( const hexapod_msgs::msg::FeetPositions::SharedPtr feet, const hexapod_msgs::msg::Pose::SharedPtr body,const hexapod_msgs::msg::LegsJoints::SharedPtr legs )
{
    double sign = -1.0;
    for( int leg_index = 0; leg_index < NUMBER_OF_LEGS; leg_index++ )
    {
        if( leg_index <= 2 )
        {
            sign = -1.0;
        }
        else
        {
            sign = 1.0;
        }

        // First calculate sinus and co-sinus for each angular axis
        Trig A = getSinCos( body->orientation.yaw + feet->foot[leg_index].orientation.yaw );
        Trig B = getSinCos( body->orientation.pitch );
        Trig G = getSinCos( body->orientation.roll );

        // Calculating totals from the feet to center of the body
        double cpr_x = feet->foot[leg_index].position.x + body->position.x - INIT_FOOT_POS_X[leg_index] - COXA_TO_CENTER_X[leg_index];

        double cpr_y = feet->foot[leg_index].position.y + sign*( body->position.y + INIT_FOOT_POS_Y[leg_index] + COXA_TO_CENTER_Y[leg_index] );

        double cpr_z = feet->foot[leg_index].position.z + body->position.z + TARSUS_LENGTH - INIT_FOOT_POS_Z[leg_index];


        // Calculation of angular matrix of body (Tait-Bryan angles Z, Y, X)
        // http://en.wikipedia.org/wiki/Euler_angles
        double body_pos_x = cpr_x - ( ( cpr_x * A.cosine * B.cosine ) +
                                      ( cpr_y * A.cosine * B.sine * G.sine - cpr_y * G.cosine * A.sine ) +
                                      ( cpr_z * A.sine * G.sine + cpr_z * A.cosine * G.cosine * B.sine  )
                                    );

        double body_pos_y = cpr_y - ( ( cpr_x * B.cosine * A.sine ) +
                                      ( cpr_y * A.cosine * G.cosine + cpr_y * A.sine * B.sine * G.sine ) +
                                      ( cpr_z * G.cosine * A.sine * B.sine - cpr_z * A.cosine * G.sine  )
                                    );

        double body_pos_z = cpr_z - ( ( -cpr_x * B.sine ) + ( cpr_y * B.cosine * G.sine ) + ( cpr_z * B.cosine * G.cosine ) );

        // Calculate foot position
        double feet_pos_x = -INIT_FOOT_POS_X[leg_index] + body->position.x - body_pos_x + feet->foot[leg_index].position.x;
        double feet_pos_y =  INIT_FOOT_POS_Y[leg_index] + sign*( body->position.y - body_pos_y + feet->foot[leg_index].position.y );
        double feet_pos_z =  INIT_FOOT_POS_Z[leg_index] - TARSUS_LENGTH + body->position.z - body_pos_z - feet->foot[leg_index].position.z;

        // Length between the Root and Foot Position ...Pythagorean theorem
        double femur_to_tarsus = sqrt( pow( feet_pos_x, 2 ) + pow( feet_pos_y, 2 ) ) - COXA_LENGTH;

        if( std::abs( femur_to_tarsus ) > ( FEMUR_LENGTH + TIBIA_LENGTH ) )
        {
            RCLCPP_FATAL(this->get_logger(), "IK Solver cannot solve a foot position that is not within leg reach!!!");
            RCLCPP_FATAL(this->get_logger(), "Shutting down so configuration can be fixed!!!");
            rclcpp::shutdown();
            break;
        }

        // Length of the sides of the triangle formed by the femur, tibia and tarsus joints.
        double side_a = FEMUR_LENGTH;
        double side_a_sqr = pow( FEMUR_LENGTH, 2 );

        double side_b = TIBIA_LENGTH;
        double side_b_sqr = pow( TIBIA_LENGTH, 2 );

        double side_c = sqrt( pow( femur_to_tarsus, 2 ) + pow( feet_pos_z, 2 ) );
        double side_c_sqr = pow( side_c, 2 );

        // We are using the law of cosines on the triangle formed by the femur, tibia and tarsus joints.
        double angle_b = acos( ( side_a_sqr - side_b_sqr + side_c_sqr ) / ( 2.0 * side_a * side_c ) );
        double angle_c = acos( ( side_a_sqr + side_b_sqr - side_c_sqr ) / ( 2.0 * side_a * side_b ) );

        // Angle of line between the femur and Tarsus joints with respect to feet_pos_z.
        double theta = atan2( femur_to_tarsus, feet_pos_z );

        // Resulting joint angles in radians.
        legs->leg[leg_index].coxa = atan2( feet_pos_x, feet_pos_y ) + INIT_COXA_ANGLE[leg_index];
        legs->leg[leg_index].femur = ( PI/2 ) - ( theta + angle_b );
        legs->leg[leg_index].tibia = ( PI/2 ) - angle_c;
        legs->leg[leg_index].tarsus = legs->leg[leg_index].femur + legs->leg[leg_index].tibia;
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Ik)