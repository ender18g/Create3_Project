#include <rtt_ur_trajectory/rtt_ur_trajectory.hpp>
#include <iomanip>
#include <rtt/internal/GlobalService.hpp>
#include <rtt_ros2/rtt_ros2.hpp>
#include <rtt_ros2_node/rtt_ros2_node.hpp>
#include <rtt_ros2_params/rtt_ros2_params.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <rtt/Component.hpp>

rtt_ur_trajectory::rtt_ur_trajectory(const std::string &name) : TaskContext(name),
                                                                fk_solver(NULL)
{

    std::cout << "rtt_ur_trajectory::rtt_ur_trajectory" << std::endl;

    // add jointstate port
    addEventPort("MsrJointState", port_msr_jointstate).doc("Measured joint state");

    // add joint trajectory port
    addPort("JointTrajectory", port_joint_trajectory).doc("Joint trajectory");


    addOperation("MoveJ", &rtt_ur_trajectory::jointCmd, this, RTT::OwnThread);
    addOperation("MoveC", &rtt_ur_trajectory::poseCmd, this, RTT::OwnThread);
    addOperation("GetJ", &rtt_ur_trajectory::jointMsr, this, RTT::OwnThread);
    addOperation("GetC", &rtt_ur_trajectory::poseMsr, this, RTT::OwnThread);

    global_ros = RTT::internal::GlobalService::Instance()->getService("ros");
    RTT::OperationCaller<bool(const std::string &)> create_node =
        global_ros->getOperation("create_named_node");
    create_node.ready();
    create_node(name);


    global_params = global_ros->getService("rosparam");
    getparam_operation = global_params->getOperation("getParameter");

    rtt_ros2_node::getNode(this)->declare_parameter("robot_description",
                                                    std::string("empty"));

    // set rml limits  {vel, acc, jerk}
    movej_limits = {0.2, 2.0, 20.0};
    movec_limits = {0.2, 2.0, 20.0};

    // initial joints for a UR5
    q_init_6 = {0.0, -1.2, 1.1, 0.4, 0.2, 0.2};

    // velocity will be between 0.0 to 1.0 for scaling
    cmd_velocity = 0.5; 
}

rtt_ur_trajectory::~rtt_ur_trajectory()
{
    if (rml)
        delete rml;
    if (ip)
        delete ip;
    if (op)
        delete op;
    if (fk_solver)
        delete fk_solver;
}

bool rtt_ur_trajectory::configureHook()
{

    std::cout << "rtt_ur_trajectory::configureHook" << std::endl;
    movement = 0;

    // get the robot description from param
    std::cout << "Awaiting robot_description param" << std::flush;
    rclcpp::ParameterValue new_string;
    std::string robot_description;

    // sleep until the robot description is not 'empty'
    while (1)
    {
        new_string = getparam_operation.call("robot_description");


        robot_description = new_string.get<std::string>();
        if (robot_description != "empty")
        {
            std::cout << "found!" << std::endl;
            break;
        }
        else
        {
            std::cout << "." << std::flush;
            sleep(1);
        }
    }
    // Robot_description received, now parse the robot description to a KDL tree
    KDL::Tree tree;
    RTT::log().setLogLevel(RTT::Logger::Info);
    if (kdl_parser::treeFromString(robot_description, tree))
    {
        RTT::log(RTT::Info) << "Description parsed to a tree" << RTT::endlog();
        if (tree.getChain("world", "tool0", chain))
        {
            // make forward kinmatics solver
            fk_solver = new KDL::ChainFkSolverPos_recursive(chain);
            // set joint limits for inverse kinematics
            KDL::JntArray qmin = KDL::JntArray(chain.getNrOfJoints());
            KDL::JntArray qmax = KDL::JntArray(chain.getNrOfJoints());
            for (size_t i = 0; i < chain.getNrOfJoints(); i++)
            {
                qmin(i) = -M_PI;
                qmax(i) = M_PI;
            }
            // make inverse kinematics velocity solver AND position solver
            ik_solver_v = new KDL::ChainIkSolverVel_pinv(chain);
            ik_solver_p = new KDL::ChainIkSolverPos_NR_JL(chain, qmin, qmax, *fk_solver, *ik_solver_v);
        }
        else
        {
            RTT::log(RTT::Error) << "Failed to parse chain between world and link"
                                 << RTT::endlog();
        }
    }
    else
    {
        RTT::log(RTT::Error) << "Failed to parse tree" << RTT::endlog();
    }
    RTT::log(RTT::Info) << "Description parsed to a chain" << RTT::endlog();

    // Init Reflexxes variables
    rml = new ReflexxesAPI(chain.getNrOfJoints(), getPeriod());
    ip = new RMLPositionInputParameters(chain.getNrOfJoints());
    op = new RMLPositionOutputParameters(chain.getNrOfJoints());

    // set initial joint states
    q = KDL::JntArray(chain.getNrOfJoints());
    qd = KDL::JntArray(chain.getNrOfJoints());
    qdd = KDL::JntArray(chain.getNrOfJoints());

    // get the q
    q = jointMsr();

    // set all q to 0
    // for (int i = 0; i < chain.getNrOfJoints(); i++)
    // {
    //     if (chain.getNrOfJoints() ==6){
    //         // likely a ur5, so give it initial joint settings
    //         q(i) = q_init_6[i];
    //     } else{
    //         q(i) = 0.0;
    //     }
    //     qd(i) = 0.0;
    //     qdd(i) = 0.0;
    // }

    // set the previous command j to the current joint position
    for(int i = 0; i < chain.getNrOfJoints(); i++){
        prev_cmd_j.push_back(q(i));
    }
    // set the previous command c to the current cartesian position
    std::vector<double> prev_cmd_c = get_rpy_xyz(poseMsr());

    // dynamically set joint_names vector
    for (int i = 0; i < chain.getNrOfSegments(); i++)
    {
        auto segment = chain.getSegment(i);
        // check this segment for a joint
        if (segment.getJoint().getTypeName() != "None")
        {
            std::string name = segment.getJoint().getName();
            joint_names.push_back(name);
            std::cout<<name<<std::endl;
        }
    }

    cart_names = {"roll", "pitch", "yaw", "x", "y", "z"};

    p_cart_names = cart_names;
    p_joint_names = joint_names;

    // make cartesion param names
    for(int i=0; i<6; i++){
        p_cart_names[i] = "c_"+std::to_string(i)+"_" + cart_names[i];
    }
    // make joint param names
    for(int i=0; i<chain.getNrOfJoints(); i++){
        p_joint_names[i] = "j_"+ std::to_string(i)+"_" + joint_names[i];
    }
    

    // ----- SET PARAMETERS ------------------------------------------
    // ---------------VELOCITY SCALE----------------
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    rcl_interfaces::msg::FloatingPointRange range;
    range.set__from_value(0.0).set__to_value(1.0);
    descriptor.description = "Velocity Scale";
    descriptor.read_only = false;
    descriptor.floating_point_range={range};
    rtt_ros2_node::getNode(this)->declare_parameter("_cmd_velocity", cmd_velocity, descriptor);

    // ---------------JOINT NAMES----------------
    for (int i = 0; i < chain.getNrOfJoints(); i++)
    {
        range.set__from_value(-M_PI).set__to_value(M_PI);
        descriptor.floating_point_range={range};
        descriptor.description = "Joint Angle";
        rtt_ros2_node::getNode(this)->declare_parameter(p_joint_names[i], prev_cmd_j[i], descriptor);
    }

    // ---------------CARTESIAN NAMES----------------
    for (int i = 0; i < cart_names.size(); i++)
    {   
        if(i<3){
            // these are angles
            range.set__from_value(-M_PI).set__to_value(M_PI);
        }else{
            // these are meters
            range.set__from_value(-2.0).set__to_value(2.0);
        }
        descriptor.floating_point_range={range};
        descriptor.description = "Cartesian Position";
        rtt_ros2_node::getNode(this)->declare_parameter(p_cart_names[i], prev_cmd_c[i], descriptor);
    }


    return true;
}

// **************START HOOK**************
bool rtt_ur_trajectory::startHook()
{
    std::cout << "rtt_ur_trajectory::startHook" << std::endl;
    jointCmd(q); 
    std::cout << "Robot Ready for GetJ, GetC, MoveJ, and MoveC commands" << std::endl;

    return true;
}

// **************UPDATE HOOK**************
void rtt_ur_trajectory::updateHook()
{
    //read the joint state
    q = jointMsr();

    // call check params to look for new MoveC, MoveJ or V commands
    check_params();


    //set current position to q
    for(int i=0; i<chain.getNrOfJoints(); i++){
        ip->CurrentPositionVector->VecData[i] = q(i);
        ip->CurrentVelocityVector->VecData[i] = qd(i);
    }
    
    int result = rml->RMLPosition(*ip, op, flags);


    //*ip->CurrentPositionVector = *op->NewPositionVector;
    *ip->CurrentVelocityVector = *op->NewVelocityVector;
    *ip->CurrentAccelerationVector = *op->NewAccelerationVector;
    // get the new position and velocity vectors
    std::vector<double> pos_vec(chain.getNrOfJoints());
    std::vector<double> vel_vec(chain.getNrOfJoints());
    op->GetNewPositionVector(pos_vec.data(), sizeof(double) * chain.getNrOfJoints());
    op->GetNewVelocityVector(vel_vec.data(), sizeof(double) * chain.getNrOfJoints());

    KDL::JntArray new_q(chain.getNrOfJoints());
    KDL::JntArray new_qd(chain.getNrOfJoints());

    // IF IN MOVE J MODE
    if (movement == 1 || movement == 0)
    {
        // set joints and velocities to the new values, they are ready to publish!
        for (int i = 0; i < chain.getNrOfJoints(); i++)
        {
            //new_q(i) = pos_vec[i];
            //new_qd(i) = vel_vec[i];
            new_q(i) = ip->TargetPositionVector->VecData[i];
            new_qd(i) = 0.0;
        }

        //update cartesian params
        update_cart_params();
    }
    else if (movement == 2)
    {
        // IF IN MOVE C MODE
        // get the frame from {rpy xyz}
        KDL::Frame frame;
        pos_vec[0]= ip->TargetPositionVector->VecData[0];
        pos_vec[1]= ip->TargetPositionVector->VecData[1];
        pos_vec[2]= ip->TargetPositionVector->VecData[2];
        pos_vec[3]= ip->TargetPositionVector->VecData[3];
        pos_vec[4]= ip->TargetPositionVector->VecData[4];
        pos_vec[5]= ip->TargetPositionVector->VecData[5];

        frame.M = KDL::Rotation::RPY(pos_vec[0], pos_vec[1], pos_vec[2]);
        frame.p = KDL::Vector(pos_vec[3], pos_vec[4], pos_vec[5]);
        
        //flip q
        // KDL::JntArray temp_q = q;
        // KDL::JntArray flipped_q = q;
        // flipped_q(0) = temp_q(2);
        // flipped_q(2) = temp_q(0);
        
        // get the inverse kinematics (and save it to new_q)
        int result_ik = ik_solver_p->CartToJnt(q, frame, new_q);

        // //unflip new_q
        // flipped_q = new_q;

        // new_q(0) = flipped_q(2);
        // new_q(2) = flipped_q(0);

        //check to see if IK solver gave bad result
        if(result_ik < 0){
            std::cout<<"IK FAILED"<<std::endl;
        }

        // set the velocity to zero (shouldnt matter for rviz)
        for (int i = 0; i < chain.getNrOfJoints(); i++)
        {
            new_qd(i) = 0.0;
        }
        // update join params
        update_joint_params();
    }

    // Check the result to see if finished
    if (result == ReflexxesAPI::RML_FINAL_STATE_REACHED && movement)
    {
        std::cout << "**** TRAJECTORY COMPLETE ****" << std::endl;
        movement = 0;
    }

    //****************WRITE TO ROS MESSAGE****************
    // pack into a joint trajectory message
    trajectory_msgs::msg::JointTrajectory joint_trajectory;
    joint_trajectory.header.stamp = rclcpp::Clock().now();
    joint_trajectory.joint_names = joint_names;
    trajectory_msgs::msg::JointTrajectoryPoint point;

    // set the position and velocity
    for (int i = 0; i < chain.getNrOfJoints(); i++)
    {
        point.positions.push_back(new_q(i));
    }
    // set the time from start
    point.time_from_start = rclcpp::Duration::from_seconds(4);
    // add the point to the trajectory
    joint_trajectory.points.push_back(point);


    // write to port
    port_joint_trajectory.write(joint_trajectory);
}

// **************STOP HOOK**************
void rtt_ur_trajectory::stopHook()
{
    std::cout << "rtt_ur_trajectory::stopHook" << std::endl;
}

// **************CLEANUP HOOK**************
void rtt_ur_trajectory::cleanupHook()
{
    std::cout << "rtt_ur_trajectory::cleanupHook" << std::endl;
}

// **************MOVE J*************************
void rtt_ur_trajectory::jointCmd(const KDL::JntArray &desired_q)
{
    std::cout << "******** Joint Command Received ********" << std::endl;
    movement = 1;
    // These are goal position/velocities
    for (size_t i = 0; i < chain.getNrOfJoints(); i++)
    {
        // set the goal position
        ip->TargetPositionVector->VecData[i] = desired_q(i);
        // always leave velocity at 0 for the goal
        ip->TargetVelocityVector->VecData[i] = 0.0;
        // set the rml limits
        ip->MaxVelocityVector->VecData[i] = movej_limits[0] * cmd_velocity;
        ip->MaxAccelerationVector->VecData[i] = movej_limits[1];
        ip->MaxJerkVector->VecData[i] = movej_limits[2];
        // set current position/velocity
        ip->CurrentPositionVector->VecData[i] = q(i);
        ip->CurrentVelocityVector->VecData[i] = 0.0;
        ip->SelectionVector->VecData[i] = true;
    }
}

// **************MOVE C*************************
void rtt_ur_trajectory::poseCmd(const KDL::Frame &desired_p)
{
    // Take in a desired frame and and move the robot
    std::cout << "******** Cartesion Command Received ********" << std::endl;
    // get the current cartesian pose in rpy xyz
    std::vector<double> current_rpy_xyz = get_rpy_xyz(poseMsr());
    // get the desired cartesian pose
    std::vector<double> desired_rpy_xyz = get_rpy_xyz(desired_p);

    
    // SETUP FINISHED, NOW PREPARE GOAL
    movement = 2;
    for (size_t i = 0; i < 6; i++)
    {
        ip->TargetPositionVector->VecData[i] = desired_rpy_xyz[i];
        // always leave velocity at 0 for the goal
        ip->TargetVelocityVector->VecData[i] = 0.0;
        // set the rml limits
        ip->MaxVelocityVector->VecData[i] = movec_limits[0] * cmd_velocity;
        ip->MaxAccelerationVector->VecData[i] = movec_limits[1];
        ip->MaxJerkVector->VecData[i] = movec_limits[2];
        // set current position/velocity
        ip->CurrentPositionVector->VecData[i] = current_rpy_xyz[i];
        ip->CurrentVelocityVector->VecData[i] = 0.0;
        ip->SelectionVector->VecData[i] = true;
    }

    // Goal has been set and limits updated
}

// *****************GET J***************************
KDL::JntArray rtt_ur_trajectory::jointMsr()
{
    // get the current joint positions from ros message
    sensor_msgs::msg::JointState jointstate;
     port_msr_jointstate.read(jointstate);

    // set the joint positions
    for (size_t i = 0; i < chain.getNrOfJoints(); i++)
    {
        std::cout << jointstate.position[i] << std::endl;
        q(i) = jointstate.position[i];
        qd(i)= jointstate.velocity[i];
    }

    // print out q on one line
    // std::cout << "q: ";
    // for (size_t i = 0; i < chain.getNrOfJoints(); i++)
    // {
    //     std::cout << q(i) << " ";
    // }
    // std::cout << std::endl;

    return q;
}

// *****************GET C***************************
KDL::Frame rtt_ur_trajectory::poseMsr()
{
    // get the current cartesian pose
    KDL::Frame p;
    fk_solver->JntToCart(q, p);
    return p;
}

std::vector<double> rtt_ur_trajectory::get_rpy_xyz(KDL::Frame f){
    // take in kdl frame and return vector rpy xyz
    std::vector<double> rpy_xyz(6);
    f.M.GetRPY(rpy_xyz[0], rpy_xyz[1], rpy_xyz[2]);
    rpy_xyz[3] = f.p.x();
    rpy_xyz[4] = f.p.y();
    rpy_xyz[5] = f.p.z();
    return rpy_xyz;
}

// *****************CHECK PARAMS********************
bool rtt_ur_trajectory::check_params()
{

    rclcpp::ParameterValue param;

    // PART 2 ASSIGNMENT 9 - Check for new JOINT positions & call MoveJ
    // check cmd joint parameter for new joint position
    std::vector<double> cmd_j_vec(p_joint_names.size());

    for(int i=0; i<p_joint_names.size(); i++){
        param = getparam_operation.call(p_joint_names[i]);
        cmd_j_vec[i] = param.get<double>();
    }

    // if this is a new command, call MoveJ
    if ((cmd_j_vec != prev_cmd_j) && movement != 2){
        // update the previous command
        prev_cmd_j = cmd_j_vec;
        // make joint array
        KDL::JntArray cmd_j_kdl = KDL::JntArray(chain.getNrOfJoints());
        for (int i = 0; i < chain.getNrOfJoints(); i++)
        {
            cmd_j_kdl(i) = cmd_j_vec[i];
        }
        // call MoveJ
        jointCmd(cmd_j_kdl);
        return 1;
    }
    
    // PART 1 ASSIGNMENT 9 - Check for new CARTESIAN positions & call MoveC
    // check cmd cartesian parameter for new cartesian position
    // NOTE This is RPY XYZ format to match my existing code
    std::vector<double> cmd_c_vec(p_cart_names.size());

    for(int i=0; i<p_cart_names.size(); i++){
        param = getparam_operation.call(p_cart_names[i]);
        cmd_c_vec[i] = param.get<double>();
    }

    // if this is a new command, call MoveC
    if ((cmd_c_vec != prev_cmd_c) && movement != 1){
        // update the previous command
        prev_cmd_c = cmd_c_vec;
        // make frame
        KDL::Frame cmd_c_kdl;
        cmd_c_kdl.M = KDL::Rotation::RPY(cmd_c_vec[0], cmd_c_vec[1], cmd_c_vec[2]);
        cmd_c_kdl.p = KDL::Vector(cmd_c_vec[3], cmd_c_vec[4], cmd_c_vec[5]);
        // call MoveC
        poseCmd(cmd_c_kdl);
        return 1;
    }

    // PART 3 Assignment 9 - Check for velocity scale command
    rclcpp::ParameterValue cmd_v;
    cmd_v = getparam_operation.call("_cmd_velocity");
    cmd_velocity = cmd_v.get<double>();
    
    // update the RML solver with new velocity
    for (int i = 0; i < chain.getNrOfJoints(); i++)
    {
        ip->MaxVelocityVector->VecData[i] = movej_limits[0] * cmd_velocity;
        // consider adding another rml solver here
    }


    // no new params to act on
    return 0;
}

// **************UPDATE PARAMS*******************
bool rtt_ur_trajectory::update_joint_params()
{
    // when moving in cartesian space, update the joint params
    for(int i=0; i<p_joint_names.size(); i++){
        rtt_ros2_node::getNode(this)->set_parameter(rclcpp::Parameter(p_joint_names[i], q(i)));

        //update prev_cmd_j
        prev_cmd_j[i] = q(i);
    }

    return 1;

}

bool rtt_ur_trajectory::update_cart_params()
{

    // when moving in joint space, update the cartesian params
    std::vector<double> current_rpy_xyz = get_rpy_xyz(poseMsr());
    for(int i=0; i<p_cart_names.size(); i++){
        rtt_ros2_node::getNode(this)->set_parameter(rclcpp::Parameter(p_cart_names[i], current_rpy_xyz[i]));
    }

    // update the prev_cmd_c
    prev_cmd_c = current_rpy_xyz;

    return 1;
}

ORO_CREATE_COMPONENT(rtt_ur_trajectory)


// TODO LIST
// 1. Check the MoveC param function------- DONE
// 2. Find the joint names from the URDF------ DONE
// 3. add another RML solver with the same size as joint number N
// 4. Remove the initial positionsd 1-6 ------ DONE
// 5. Add a velocity parameter------ DONE
// 6. Create RQT GUI------- DONE
// 7. Sync up the params at each step
// 8. Launch RQT GUI in launch file

