//
// Created by idriver on 17-3-10.
//

#include "omegapathplanner.h"

omegapathplanner::omegapathplanner(ros::NodeHandle mh)
{
    dwa = std::unique_ptr<dwaplanner>(new dwaplanner(mh));
    heuristic = std::unique_ptr<sehs>(new sehs);
    gt = std::unique_ptr<geotool>(new geotool);

    // subscribe
    sub_Obj = mh.subscribe(SUB_TOPIC_OBJ, 10, &omegapathplanner::chatterCallbackObj, this);
    sub_Vsp = mh.subscribe(SUB_TOPIC_VSP, 10, &omegapathplanner::chatterCallbackVsp, this);
    sub_Vap = mh.subscribe(SUB_TOPIC_VAP, 10, &omegapathplanner::chatterCallbackVap, this);
    sub_App = mh.subscribe(SUB_TOPIC_APP, 10, &omegapathplanner::chatterCallbackApp, this);
    sub_User = mh.subscribe(SUB_TOPIC_USER, 10, &omegapathplanner::chatterCallbackUser, this);
    sub_Traffic = mh.subscribe(SUB_TOPIC_TRAFFIC, 10, &omegapathplanner::chatterCallbackTraffic, this);
    sub_Pos = mh.subscribe(SUB_TOPIC_LOCPOS, 10, &omegapathplanner::chatterCallbackLocpos, this);
    sub_Tag = mh.subscribe(SUB_TOPIC_LOCTAG, 10, &omegapathplanner::chatterCallbackLoctag, this);
    sub_Obs = mh.subscribe(SUB_TOPIC_OBSAVOID, 10, &omegapathplanner::chatterCallbackObsavoid, this);
    sub_Dec = mh.subscribe(SUB_TOPIC_DECISION, 10, &omegapathplanner::chatterCallbackDecision, this);
    sub_Pred = mh.subscribe(SUB_TOPIC_PREDICT, 10, &omegapathplanner::chatterCallbcalPredict, this);
    sub_Stcell = mh.subscribe("ivmapstaticcell", 10, &omegapathplanner::chatterCallbackObjstcell, this);
    sub_Navigation = mh.subscribe(SUB_TOPIC_NAVIGATION, 10, &omegapathplanner::chatterCallbackNavigation, this);
    sub_Move = mh.subscribe(SUB_TOPIC_MOVE, 10, &omegapathplanner::chatterCallbackMove, this);

    // advertise
    pub_Path = mh.advertise<ivpathplanner::ivmsgpath>(PUB_TOPIC_IAPATH, 10);
    pub_Feedback = mh.advertise<ivpathplanner::ivmsgpathfeedback>(PUB_TOPIC_FEEDBACK, 10);
    pub_motiondebug = mh.advertise<ivpathplanner::motiondebug>(PUB_TOPIC_MOTIONDEBUG, 10);
    pub_deliver = mh.advertise<ivpathplanner::deliverinfo>(PUB_TOPIC_DELIVERINFO, 10);
    pub_carstateshow = mh.advertise<ivpathplanner::carstateshow>("carstateshow", 10);

    for (int i = 0; i < MAX_SIZE; ++i)
    {
        limit_set <<= 1;
        limit_set.set(0);
    }

    // Parameters initialize
    ros::topic::waitForMessage<ivmap::ivmapmsglocpos>("ivmaplocpos");
    InitParams();
    int node_id = 19;
    mcuMonitor = new Monitor(node_id);
}

omegapathplanner::~omegapathplanner()
{
    delete mcuMonitor;
    mcuMonitor = NULL;
}

void omegapathplanner::InitParams()
{
    ros::NodeHandle mh;
    // navigationPlanEnable = true;
    navigationPlanEnable = false;
    obsavoid.enable = 0;
    obsavoid.obsid = 0;
    numPathSegs = 10;
    limit_count = 0;
    reversing_count = 0;
    failure_count = 0;
    current_seg = 0;

    limit_set.reset();

    mh.param("numPathSegs", numPathSegs, numPathSegs);
    test_segid = 1001;
    mh.param("test_segid", test_segid, test_segid);
    test_enable = false;
    mh.param("test_enable", test_enable, test_enable);
    std::string routemap;
    mh.param("routemap", routemap, routemap);

    GetNoAvoidArea(routemap);

    if (false == test_enable)
    {
        RefPath.LoadAllMaps(numPathSegs);
        RefPathRaw.LoadAllMaps(numPathSegs);
        refCheck.LoadAllMaps(numPathSegs);
    }
    else
    {
        RefPath.LoadAllMaps(test_segid);
        RefPathRaw.LoadAllMaps(test_segid);
        refCheck.LoadAllMaps(numPathSegs);
    }

    MotionPlan.Init_motionplanner();

    pathfeedbackmsg.freedrive = 1;

    //Freespace boundary
    delivermsg.reachedgoal = 0;
    reachable_flag = true;
    predict_flag = false;
    ivmap_flag = false;
    decision_flag = false;
    reversing_flag = false;
    ifchange_flag = false;

    offset_distance = 0.0;

    previous_state = std::make_pair(-1, "initial_state");

    ego_time = ros::Time::now();
    ivmap_time = ros::Time::now();
    predict_time = ros::Time::now();
    decision_time = ros::Time::now();
    current_time = ros::Time::now();

    segSequency.clear();
    blind_area.clear();
    NavigationPlan();
}

void omegapathplanner::NavigationPlan()
{
    if (true == navigationPlanEnable)
    {
        navigationPlanEnable = false;
        // segSequency.push_back(0);
        // segSequency.push_back(1);
        // current_seg = 1;
        if (!segSequency.empty())
        {
            RefPath.PathSegsToSmoothPath(segSequency);
            RefPathRaw.PathSegsToSmoothPath(segSequency);
        }
    }
    if (true == test_enable)
    {
        segSequency.clear();
        segSequency.push_back(test_segid - 1);
        RefPath.PathSegsToSmoothPath(segSequency);
        RefPathRaw.PathSegsToSmoothPath(segSequency);
    }
}

void omegapathplanner::LoadLocalRefPath()
{
    NavigationPlan();
    optionPaths.paths.clear();
    RefPath.LocalRefpathBuild(ivlocpos, 28, 0.0, LocalRefPath, 10);
    RefPathRaw.LocalRefpathBuild(ivlocpos, 28, 0.0, LocalRefPathRaw, 10);
    optionPaths.paths.push_back(LocalRefPath);
}

void omegapathplanner::FreeDriveCheck()
{
    int collisionflag = 0;

    sVehicleElem vehiclemodel;
    vehiclemodel.frontoverhang = 1.3;
    vehiclemodel.backoverhang = 0.3;
    vehiclemodel.halfwheeltrack = 0.50;
    collisionflag = dwa->collisiondetection(LocalRefPath, rawObsTree, vehiclemodel);

    if (1 == collisionflag)
    {
        pathfeedbackmsg.freedrive = 0;
        reachable_flag = false;
        collid_obstacles = rawObsTree;
        // collid_obstacles = dilatedObsTree;
    }
    else
    {
        pathfeedbackmsg.freedrive = 1;
        reachable_flag = true;
    }
}

void omegapathplanner::EditMap()
{
    /* Follow the reference path */
    if (ID_SelectPath == 0)
    {
        RefPath.VelocityUpdate(IAPath);
        RefPath.MapRecovery();
    }
    else
    {
        while (obsavoid.obsid > pathfeedbackmsg.avoidobs)
        {
            pathfeedbackmsg.avoidobs += 1;
        }
        RefPath.LocalPathToMap(ivlocpos, IAPath);
    }
}

void omegapathplanner::PubMessage()
{
    ivpathplanner::ivmsgpath pubedPath;
    ivpathplanner::motionpoint tmp;

    float len = 0.0;
    float tempdis = 0.0;

    offset_distance = 0.0;

    pubedPath.currentpathdirection = 0;
    if (!IAPath.points.empty())
    {
        for (int i = 0; i < IAPath.points.size() - 1; ++i)
        {

            tempdis = std::hypot(IAPath.points.at(i + 1).x - IAPath.points.at(i).x,
                                 IAPath.points.at(i + 1).y - IAPath.points.at(i).y);
            len += tempdis;
            if (len <= 8.0)
            {
                offset_distance += IAPath.points.at(i).y;
            }

            pubedPath.points.push_back(IAPath.points.at(i));

            /*There is a virtual stopping motion point*/
            if (true == stopping_point.stopflag)
            {
                if (std::hypot(IAPath.points.at(i).xg - stopping_point.points.front().xg,
                               IAPath.points.at(i).yg - stopping_point.points.front().yg) < 0.5)
                {
                    break;
                }
            }
        }
    }

    /*
    if (len <= 0.4){
        for (auto& a : pubedPath.points){
            a.velocity = 0.0;
            a.emergency_flag = true;
        }
    }else if (len <= 20.0){
        for (auto& a : pubedPath.points){
            if (fabs(a.velocity) <= 1e-3)
            {
                continue;
            }
	    a.velocity = 0.4 + 0.2 * len / 20.0;
        }
    }
*/

    if (true == reversing_flag)
    {
        pubedPath.points.clear();
        IAPath.points.clear();
        for (auto a : reversePath.points)
        {
            tmp.x = a.x;
            tmp.y = a.y;
            tmp.angle = a.angle;
            tmp.velocity = a.velocity;
            if (true == emergencyFlag)
            {
                tmp.velocity = 0.0;
                tmp.emergency_flag = true;
            }
            pubedPath.points.push_back(tmp);
        }
        pubedPath.currentpathdirection = 1;
    }

    pub_Path.publish(pubedPath);
    pub_Feedback.publish(pathfeedbackmsg);
    pub_motiondebug.publish(motionvars);

    static int pubCount = 0;
    pubCount++;
    delivermsg.reachedgoal = reachedGoal();
    if (40 == pubCount)
    {
        pub_deliver.publish(delivermsg);
        pubCount = 0;
    }
}

int omegapathplanner::reachedGoal()
{
    if (true == reversing_flag)
    {
        return 0;
    }

    if (true == IAPath.points.empty())
    {
        return delivermsg.reachedgoal;
    }
    else
    {
        if (std::hypot(ivlocpos.xg - IAPath.points.back().xg,
                       ivlocpos.yg - IAPath.points.back().yg) <= 3 &&
            ivvap.v < 0.2)
        {
            return current_seg;
        }
    }
    return 0;
}

bool omegapathplanner::emergencyStop(bool backward)
{
    ivpathplanner::pathpoint temp;
    ivpathplanner::path lpath, rpath;
    bool lflag = false;
    bool rflag = false;
    static int safe_count = 0;
    static bool triggerd = false;
    static float braking_distance = 0.0;
    sVehicleElem vehiclemodel;
    vehiclemodel.frontoverhang = 0.60;
    vehiclemodel.backoverhang = 0.30;
    vehiclemodel.halfwheeltrack = 0.40;
    /*forwarding driving*/
    if (false == backward)
    {
        float t_delay = 0.6;
        float decrease_rate = 0.8;
        float step = 0.0;
        if (!triggerd)
        {
            braking_distance = ivvap.v * t_delay + std::pow(ivvap.v, 2) / (2 * decrease_rate);
        }
        while (step < braking_distance)
        {
            temp.x = 1.3 + step;
            temp.y = 0.0;
            temp.angle = 0.0;
            rpath.points.push_back(temp);
            step += 0.5;
        }
        temp.x = 1.3 + braking_distance;
        temp.y = 0.0;
        temp.angle = 0.0;
        rpath.points.push_back(temp);

        temp.x = 1.3;
        temp.y = 0.0;
        temp.angle = 0.0;
        lpath.points.push_back(temp);
        temp.x = 0.4;
        temp.y = 0.0;
        temp.angle = 0.0;
        lpath.points.push_back(temp);

        rflag = dwa->collisiondetection(rpath, emergencyObsTree, vehiclemodel);
        if (true == rflag)
        {
            triggerd = true;
            safe_count = 0;
            return true;
        }

        if (safe_count < 3)
        {
            safe_count++;
            return true;
        }
        triggerd = false;
    }
    /*backward driving*/
    else
    {
        temp.x = -0.4;
        temp.y = 0.0;
        temp.angle = 180.0;
        lpath.points.push_back(temp);
        temp.x = -0.8;
        temp.y = 0.0;
        temp.angle = 180.0;
        lpath.points.push_back(temp);
    }

    lflag = dwa->collisiondetection(lpath, emergencyObsTree, vehiclemodel);
    if (true == lflag)
    {
        return true;
    }
    return false;
}

void omegapathplanner::run()
{
    ros::Rate rate(HZ);
    while (ros::ok())
    {
        ros::spinOnce();

        /*preprocess*/
        sorted_tree().swap(emergencyObsTree);
        sorted_tree().swap(rawObsTree);
        sorted_tree().swap(dilatedObsTree);
        sorted_tree().swap(collid_obstacles);
        dwa->obstaclemodify(ivpredict, emergencyObsTree, rawObsTree, dilatedObsTree, ivvap.v);
        dwa->obstaclemodify(ivstobj, emergencyObsTree, rawObsTree, dilatedObsTree);

        tmp_decision = decision;
        emergencyFlag = false;
        area_avoid_ = true;

        bool avoiding_flag = false;

        int turning = 0; // -1 left, +1 right;

        ego_time = ros::Time::now();

        LoadLocalRefPath();
        OptionAvoid();

        if (area_avoid_ && 1 == decision.refpathsource && !reversing_flag)
        {
            optionPaths.roadsource = 1;

            FreeDriveCheck();
            //updateBlindTree();

            turning = dwa->TurningBehavior(ivlocpos, LocalRefPath, stopping_point);

            /*normal avoiding obstacle process*/
            if (obsavoid.enable == 1 && obsavoid.obsid > pathfeedbackmsg.avoidobs)
            {
                ros::Time timePre = ros::Time::now();
                reachable_flag = false;

                /*Essential for the stability*/
                ivpathplanner::path updatepath;
                updatepath = dwa->pathinterface(ivlocpos, LocalRefPath, LocalRefPathRaw, collid_obstacles);
                //updatepath = heuristic->planning(ivlocpos, LocalRefPath, LocalRefPathRaw, collid_obstacles);

                if (!updatepath.points.empty())
                {
                    failure_count = 0;
                    reachable_flag = true;
                    avoiding_flag = true;
                    optionPaths.paths.push_back(updatepath);
                }
                else
                {
                    if (fabs(ivvap.v) <= 0.1)
                    {
                        failure_count++;
                    }
                }
                double timecost = (ros::Time::now() - timePre).toSec();
            }

            /*Judge if the car deviate the raw road, means there is a new
              obstacle avoiding road*/

            /*give way process*/
            // GivewayProcess();
        }

        ReversingProcess();

        emergencyFlag = emergencyStop(reversing_flag);
        if (true == emergencyFlag && !IAPath.points.empty())
        {
            failure_count++;
            tmp_decision.vellimit = 0.0;
            reachable_flag = false;
        }

        HMIProcess(avoiding_flag, turning);

        /*************motionplanner**************/

        /*
        * select one path from the optional paths
        * Inputs: optionPaths
        * Outputs: IAPath, the No. of selected path
        */

        ID_SelectPath = 0;
        if (optionPaths.paths.size() > 0 && !reversing_flag)
        {
            if (optionPaths.paths[0].points.size() > 0)
            {
                MotionPlan.Basesignals_get(ivvap);
                MotionPlan.Decision_get(tmp_decision, reachable_flag, emergencyFlag, avoiding_flag);
                MotionPlan.Velocitygenbasedroad(optionPaths);
                ivmap::ivmapmsgbdcell ivobjbc;
                MotionPlan.Velocitygenbasedobs(MotionPlan.option_pathpoints, MotionPlan.obs, ivobjbc);
                ID_SelectPath = MotionPlan.PathSelect(MotionPlan.option_pathpoints);
                MotionPlan.PathGeneration(MotionPlan.option_pathpoints);
                IAPath = MotionPlan.finalpath;
                motionvars = MotionPlan.motionvars;
                if (decision.refpathsource == 1)
                {
                    EditMap();
                }
            }
        }

        if (delivermsg.reachedgoal != 0)
        {
            if (ifchange_flag)
            {
                ifchange_flag = false;
                delivermsg.reachedgoal = 0;
            }
            else
            {
                IAPath.points.clear();
            }
        }

        if (reachable_flag == true)
        {
            failure_count = 0;
        }

        PubMessage();
        FailSafeMode();

        rate.sleep();
    }
}

void omegapathplanner::HMIProcess(bool avoiding_flag, int turning)
{
    int direction = 0; // -1 left, +1 right;
    std::map<int, std::string, greater<int>> current_state;

    current_state.insert(std::make_pair(0, "trafic_free"));
    if (true == emergencyFlag)
    {
        current_state.insert(std::make_pair(7, "emergency_stop"));
    }
    if (offset_distance > 1.0)
    {
        //left
        direction = -1;
    }
    else if (offset_distance < -1.0)
    {
        //right
        direction = 1;
    }

    if (true == avoiding_flag)
    {
        if (-1 == direction)
        {
            current_state.insert(std::make_pair(5, "left_avoiding"));
        }
        else if (1 == direction)
        {
            current_state.insert(std::make_pair(6, "right_avoiding"));
        }
    }
    else
    {
        if (delivermsg.reachedgoal != 0)
        {
            current_state.insert(std::make_pair(1, "reached_goal"));
        }
        else
        {
            if (-1 == turning)
            {
                current_state.insert(std::make_pair(2, "left_turning"));
            }
            else if (1 == turning)
            {
                current_state.insert(std::make_pair(3, "right_turning"));
            }
        }
    }
    InteractionWindow(previous_state, current_state);
    if (!current_state.empty())
    {
        previous_state = *(current_state.begin());
    }
    return;
}

void omegapathplanner::ReversingProcess()
{
    if (failure_count > 40 && 0 == delivermsg.reachedgoal)
    {
        reversing_flag = true;
    }

    if (failure_count > 90 || reversing_count > 210)
    {
        reversing_flag = false;
        failure_count = 0;
        reversing_count = 0;
        reversePath.points.clear();
    }

    if (true == reversing_flag)
    {
        //reversePath = dwa->ReverseInterface(ivlocpos, LocalRefPathRaw, emergencyObsTree);
        reversePath = dwa->ReverseInterface(ivlocpos, reversePath, emergencyObsTree);
        if (reversePath.points.empty())
            failure_count++;

        reversing_count++;
    }

    if (reversing_count >= 40 && reversing_count % 40 == 0)
    {
        ivpathplanner::path updatepath = dwa->pathinterface(ivlocpos, LocalRefPath, LocalRefPathRaw, rawObsTree);
        if (!updatepath.points.empty())
        {
            optionPaths.paths.push_back(updatepath);
            reversing_flag = false;
            failure_count = 0;
            reversing_count = 0;
            reversePath.points.clear();
        }
    }
    return;
}

void omegapathplanner::GivewayProcess()
{
    /*give way process*/
    bool limit_flag = false;
    ivpathplanner::path slowing_down;
    if (LocalRefPath.points.size() > 25 && LocalRefPathRaw.points.size() > 25)
    {
        ivmap::ivmapmsgobj dynamic_obj;
        for (auto a : ivobj.obj)
        {
            if (fabs(a.vabs) < 3.0)
            {
                continue;
            }
            float thw = std::hypot(a.x, a.y) / fabs(a.vabs);
            if (a.vabs > 0.0)
            {
                if (a.x < 2.0)
                {
                    if (thw < 10.0)
                    {
                        dynamic_obj.obj.push_back(a);
                    }
                }
            }
            else
            {
                if (a.x > 1.0)
                {
                    if (thw < 10.0)
                    {
                        dynamic_obj.obj.push_back(a);
                    }
                }
            }
        }
        std::vector<float> vec_offset;
        vec_offset.resize(dynamic_obj.obj.size(), 100.0);
        int min_size = std::min(LocalRefPath.points.size(), LocalRefPathRaw.points.size());
        float max_offset = 0.0;
        float len = 0.0;
        for (int i = 0; i < min_size - 1; ++i)
        {
            /*Get the collision avoiding offset*/
            float dis = std::hypot(LocalRefPath.points.at(i).y - LocalRefPathRaw.points.at(i).y,
                                   LocalRefPath.points.at(i).x - LocalRefPathRaw.points.at(i).x);
            if (max_offset < dis)
            {
                max_offset = dis;
            }
            /*Get the slowing down path*/
            float temp_len = std::hypot(LocalRefPath.points.at(i).x - LocalRefPath.points.at(i + 1).x,
                                        LocalRefPath.points.at(i).y - LocalRefPath.points.at(i + 1).y);
            len += temp_len;
            if (len < 8.0)
            {
                slowing_down.points.push_back(LocalRefPath.points.at(i));
            }
            /*Calculate the distance between the dynamic objects and the raw referance path*/
            for (int j = 0; j < dynamic_obj.obj.size(); ++j)
            {
                float dis_offset = std::hypot(LocalRefPathRaw.points.at(i).xg - dynamic_obj.obj.at(j).xg,
                                              LocalRefPathRaw.points.at(i).yg - dynamic_obj.obj.at(j).yg);
                if (dis_offset < vec_offset.at(j))
                {
                    vec_offset.at(j) = dis_offset;
                }
            }
        }

        float initial_offset = std::hypot(LocalRefPath.points.front().y - LocalRefPathRaw.points.front().y,
                                          LocalRefPath.points.front().x - LocalRefPathRaw.points.front().x);
        if (max_offset > 0.8 && initial_offset > 0.1 &&
            initial_offset < 1.6 && fabs(initial_offset - max_offset) > 0.1)
        {
            /*Get the safe zone and risky zone*/
            for (int i = 0; i < vec_offset.size(); ++i)
            {
                if (vec_offset.at(i) - dynamic_obj.obj.at(i).width * 0.5 - 1.0 < max_offset)
                {
                    limit_flag = true;
                    break;
                }
            }
            limit_set <<= 1;
            if (true == limit_flag)
            {
                limit_set.set(0);
            }
            else
            {
                limit_set.reset(0);
            }

            static bool keep_flag = false;
            if (limit_set.any())
            {
                float ratio = limit_set.count() / limit_set.size();
                if (ratio >= 0.9)
                {
                    limit_count = 0;
                    keep_flag = true;
                    tmp_decision.vellimit = 0.0;
                }
            }

            if (keep_flag == true)
            {
                if (limit_count <= 25)
                {
                    limit_count++;
                    tmp_decision.vellimit = 0.0;
                }
                else
                {
                    limit_count = 0;
                    keep_flag = false;
                }
            }
        }
    }

    /*Slowing down process*/
    sVehicleElem vehiclemodel;
    vehiclemodel.frontoverhang = 1.3;
    vehiclemodel.backoverhang = 0.4;
    vehiclemodel.halfwheeltrack = 1.0;
    int collisionflag = dwa->collisiondetection(slowing_down, emergencyObsTree, vehiclemodel);
    if (1 == collisionflag)
    {
        if (tmp_decision.vellimit > 0.7)
            tmp_decision.vellimit = 0.7;
    }

    return;
}

void omegapathplanner::InteractionWindow(std::pair<int, std::string> previous, std::map<int, std::string, greater<int>> current)
{
    static int delay_count = 0;
    static int running_count = 0;
    static std::pair<int, std::string> running_state;

    std::pair<int, std::string> priority;
    bool activated = false;
    sendCarStateShowToIVACTUTER(cmdArray[0], true, 0);
    sendCarStateShowToIVACTUTER(cmdArray[7], true, 0);
    if (current.empty())
    {
        return;
    }
    else
    {
        priority = *(current.begin());
        switch (priority.first)
        {
        case 0:
        {
            //"trafic_free"
            sendCarStateShowToIVACTUTER("ledcmd", true, 1);
            break;
        }
        case 1:
        {
            //"reached_goal"
            sendCarStateShowToIVACTUTER("ledcmd", true, 6);
            break;
        }
        case 2:
        {
            //"left_turning"
            sendCarStateShowToIVACTUTER(cmdArray[1], true, 0);
            sendCarStateShowToIVACTUTER("ledcmd", true, 8);
            break;
        }
        case 3:
        {
            //"right_turning"
            sendCarStateShowToIVACTUTER(cmdArray[2], true, 0);
            sendCarStateShowToIVACTUTER("ledcmd", true, 7);
            break;
        }
        case 4:
        {
            sendCarStateShowToIVACTUTER(cmdArray[4], true, 0);
            sendCarStateShowToIVACTUTER("ledcmd", true, 5);
            break;
        }
        case 5:
        {
            //"left_avoiding"
            running_count = 100;
            activated = true;
            sendCarStateShowToIVACTUTER(cmdArray[1], true, 0);
            sendCarStateShowToIVACTUTER("ledcmd", true, 3);
            break;
        }
        case 6:
        {
            //"right_avoiding"
            running_count = 100;
            activated = true;
            sendCarStateShowToIVACTUTER(cmdArray[2], true, 0);
            sendCarStateShowToIVACTUTER("ledcmd", true, 4);
            break;
        }
        case 7:
        {
            //"emergency_stop"
            sendCarStateShowToIVACTUTER(cmdArray[4], true, 0);
            sendCarStateShowToIVACTUTER(cmdArray[8], true, 0);
            sendCarStateShowToIVACTUTER("ledcmd", true, 5);
            break;
        }
        default:
            break;
        }
        if (running_count > 0)
            running_count--;

        //initialization
        if (-1 == previous.first || true == activated)
        {
            sendCarStateShowToIVACTUTER("send", true, 0);
            running_state = priority;
            return;
        }

        if (0 == priority.first)
        {
            if (previous.first != priority.first)
            {
                delay_count = 0;
            }
            else
            {
                if (delay_count > 100)
                {
                    sendCarStateShowToIVACTUTER("send", true, 0);
                    running_state = priority;
                    delay_count = 0;
                }
                delay_count++;
            }
        }
        else
        {
            if (0 == running_count && running_state.first != priority.first)
            {
                sendCarStateShowToIVACTUTER("send", true, 0);
                running_state = priority;
            }
        }
    }
    return;
}

void omegapathplanner::FailSafeMode()
{
    float time_threshold = 0.05;
    ros::Duration duration = ros::Duration(0.0);
    if (false == decision_flag)
    {
        mcuMonitor->sendWarnning(2, 1);
    }
    if (false == predict_flag)
    {
        mcuMonitor->sendWarnning(2, 4);
    }

    duration = current_time - decision_time;
    if (fabs(duration.toSec()) > time_threshold + 0.1)
    {
        mcuMonitor->sendWarnning(3, 3);
    }
    else if (fabs(duration.toSec()) > time_threshold + 0.05)
    {
        mcuMonitor->sendWarnning(3, 2);
    }
    else if (fabs(duration.toSec()) > time_threshold + 0.02)
    {
        mcuMonitor->sendWarnning(3, 1);
    }
    else if (fabs(duration.toSec()) > time_threshold + 0.01)
    {
        mcuMonitor->sendWarnning(3, 0);
    }
    if (fabs(duration.toSec()) > time_threshold + 0.5)
    {
        decision_flag = false;
        mcuMonitor->sendWarnning(2, 1);
    }
    if (fabs(duration.toSec()) > time_threshold + 0.1)
    {
        mcuMonitor->sendWarnning(3, 7);
    }
    else if (fabs(duration.toSec()) > time_threshold + 0.05)
    {
        mcuMonitor->sendWarnning(3, 6);
    }
    else if (fabs(duration.toSec()) > time_threshold + 0.02)
    {
        mcuMonitor->sendWarnning(3, 5);
    }
    else if (fabs(duration.toSec()) > time_threshold + 0.01)
    {
        mcuMonitor->sendWarnning(3, 4);
    }
    if (fabs(duration.toSec()) > time_threshold + 0.5)
    {
        predict_flag = false;
        mcuMonitor->sendWarnning(2, 4);
    }

    duration = current_time - ego_time;
    if (fabs(duration.toSec()) > time_threshold + 0.1)
    {
        mcuMonitor->sendWarnning(5, 3);
    }
    else if (fabs(duration.toSec()) > time_threshold + 0.05)
    {
        mcuMonitor->sendWarnning(5, 2);
    }
    else if (fabs(duration.toSec()) > time_threshold + 0.02)
    {
        mcuMonitor->sendWarnning(5, 1);
    }
    else if (fabs(duration.toSec()) > time_threshold + 0.01)
    {
        mcuMonitor->sendWarnning(5, 0);
    }
    current_time = ros::Time::now();
}

/**********************************************part 0: callback functions*****************************************/

void omegapathplanner::chatterCallbackObj(const ivmap::ivmapmsgobj::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivobj = *msg;
}

void omegapathplanner::chatterCallbackObjstcell(const ivmap::ivmapmsgstcell::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivstobj = *msg;
}

void omegapathplanner::chatterCallbackVsp(const ivmap::ivmapmsgvsp::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivvsp = *msg;
}

void omegapathplanner::chatterCallbackVap(const ivmap::ivmapmsgvap::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivvap = *msg;
}

void omegapathplanner::chatterCallbackApp(const ivmap::ivmapmsgapp::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivapp = *msg;
}

void omegapathplanner::chatterCallbackUser(const ivmap::ivmapmsguserfun::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivuserfun = *msg;
}

void omegapathplanner::chatterCallbackTraffic(const ivmap::ivmapmsgtraffic::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivtraffic = *msg;
}

void omegapathplanner::chatterCallbackLocpos(const ivmap::ivmapmsglocpos::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivlocpos = *msg;
}

void omegapathplanner::chatterCallbackLoctag(const ivmap::ivmapmsgloctag::ConstPtr &msg)
{
    ivmap_flag = true;
    ivmap_time = ros::Time::now();
    ivloctag = *msg;
}

void omegapathplanner::chatterCallbackObsavoid(const ivdecision::ivmsgobsavoid::ConstPtr &msg)
{
    decision_flag = true;
    decision_time = ros::Time::now();
    obsavoid = *msg;
}

void omegapathplanner::chatterCallbackDecision(const ivdecision::ivmsgdecision::ConstPtr &msg)
{
    decision_flag = true;
    decision_time = ros::Time::now();
    decision = *msg;
}

void omegapathplanner::chatterCallbcalPredict(const ivpredict::ivmsgpredict::ConstPtr &msg)
{
    predict_flag = true;
    predict_time = ros::Time::now();
    ivpredict = *msg;
    MotionPlan.obs = *msg;
}

void omegapathplanner::chatterCallbackNavigation(const ivmap::ivmapmsgnavigation::ConstPtr &msg)
{
    ivmap::ivmapmsgnavigation ivnavigation;
    ivnavigation = *msg;
    if (true == ivnavigation.navigationflag)
    {
        segSequency.clear();
        for (int i = 0; i < ivnavigation.roadvectors.size(); i++)
        {
            segSequency.push_back(ivnavigation.roadvectors[i].roadvector[0].roadid - 1);
        }
        navigationPlanEnable = true;
    }
}

void omegapathplanner::chatterCallbackMove(const ivapp::ivappjustmove::ConstPtr &msg)
{
    ivjustmove = *msg;
    ifchange_flag = true;
    switch (ivjustmove.moveflag)
    {
    case 1:
        navigationPlanEnable = true;
        segSequency.clear();
        segSequency.push_back(0);
        break;
    case 2:
        navigationPlanEnable = true;
        segSequency.clear();
        segSequency.push_back(1);
        break;
    default:
        break;
    }
    current_seg = ivjustmove.moveflag;
}

int omegapathplanner::sendCarStateShowToIVACTUTER(const char *cmdString, bool showCmd, int ledcmd)
{
    const char *cmdArray[9] = {"restart", "TurnLight_Left", "TurnLight_Right", "HeadLight", "BrakeLight", "ReversingLight", "DoubleFlashLight", "TailLight", "Horn"};
    static bool cmdValue[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
    static int leddisplayvalue = 0;
    if (!strcmp(cmdString, "ledcmd"))
    {
        leddisplayvalue = ledcmd;
        return 0;
    }
    for (int i = 0; i < 9; i++)
    {
        if (!strcmp(cmdString, cmdArray[i]))
        {
            cmdValue[i] = showCmd;
            break;
        }
    }
    if (cmdValue[0] == true)
        for (int i = 0; i < 9; i++)
            cmdValue[i] = 0;
    if (!strcmp(cmdString, "send") && showCmd == true)
    {
        //printf("----hour---%d\n",cmdValue[8]);
        ivpathplanner::carstateshow pubcarshowstatemsg;
        //   pubcarshowstatemsg.openLockcmd = cmdValue[0];//turn left:1.....tern right:2....
        pubcarshowstatemsg.turnlightleft = cmdValue[1];
        pubcarshowstatemsg.turnlightright = cmdValue[2];
        pubcarshowstatemsg.headlight = cmdValue[3];
        time_t now = time(0);
        tm *ltm = localtime(&now);
        if (18 <= ltm->tm_hour || ltm->tm_hour <= 6)
        {
            pubcarshowstatemsg.headlight = 1;
        }
        pubcarshowstatemsg.brakelight = cmdValue[4];
        pubcarshowstatemsg.reversinglight = cmdValue[5];
        pubcarshowstatemsg.doubleflashlight = cmdValue[6];
        pubcarshowstatemsg.taillight = cmdValue[7];
        pubcarshowstatemsg.horn = 0;
        // pubcarshowstatemsg.horn = cmdValue[8];
        pubcarshowstatemsg.leddisplaycmd = leddisplayvalue;
        pub_carstateshow.publish(pubcarshowstatemsg);
    }
    return 0;
}

double omegapathplanner::keepDecimal(const double source, const int reserved) const
{
    double power = std::pow(10, reserved);
    double carry = std::pow(0.1, reserved);
    double ifnag = 1.0;
    double rtn = 0.0;

    int new_source = std::fabs(source * power);

    int int_part = new_source / power;
    int dec_part = new_source % (int)power;

    if (source < 0)
        ifnag = -1;

    if (((double)std::fabs(source * power) - new_source) < 0.5)
        carry = 0;

    rtn = ifnag * (int_part + (double)dec_part / power + carry);

    return rtn;
}

Site omegapathplanner::tfBaseLinkToMap(const ivmap::ivmapmsglocpos &ivlocpos, const Site &obj_under_vcs) const
{
    sPointOfVCS obj_pos;
    obj_pos.x = obj_under_vcs.x;
    obj_pos.y = obj_under_vcs.y;

    sPointOfGCCS car_pos;
    car_pos.xg = ivlocpos.xg;
    car_pos.yg = ivlocpos.yg;
    car_pos.angle = ivlocpos.angle;

    sPointOfGCCS rtn = gt->VCS2GCCS(car_pos, obj_pos);

    double tmp_x = keepDecimal(rtn.xg);
    double tmp_y = keepDecimal(rtn.yg);

    return Site(tmp_x, tmp_y);
}

Site omegapathplanner::tfMapToBaseLink(const ivmap::ivmapmsglocpos &ivlocpos, const Site &obj_under_gccs) const
{
    sPointOfGCCS obj_pos;
    obj_pos.xg = obj_under_gccs.x;
    obj_pos.yg = obj_under_gccs.y;
    obj_pos.angle = 0.0;

    sPointOfGCCS car_pos;
    car_pos.xg = ivlocpos.xg;
    car_pos.yg = ivlocpos.yg;
    car_pos.angle = ivlocpos.angle;

    sPointOfVCS rtn = gt->GCCS2VCS(car_pos, obj_pos);

    double tmp_x = keepDecimal(rtn.x);
    double tmp_y = keepDecimal(rtn.y);

    return Site(tmp_x, tmp_y);
}

void omegapathplanner::updateBlindTree()
{
    for (auto it = emergencyObsTree.begin(); it != emergencyObsTree.end(); ++it)
    {
        for (auto itr = (it->second).begin(); itr != (it->second).end(); ++itr)
        {
            double tmp_x = keepDecimal(itr->x);
            double tmp_y = keepDecimal(itr->y);

            if (tmp_x > 0 && tmp_x < 2.1 && std::fabs(tmp_y) < 1)
            {
                ROS_INFO_STREAM("[fyn]:old_obj is:"
                                << "(" << tmp_x << "," << tmp_y << ")");
                Site obj_under_vcs(tmp_x, tmp_y);
                Site obj_under_gccs = tfBaseLinkToMap(ivlocpos, obj_under_vcs);
                blind_area.emplace(obj_under_gccs, ros::Time::now());
            }
            else
                continue;
        }
    }

    for (auto it = blind_area.begin(); it != blind_area.end(); ++it)
    {
        ros::Duration duration = ros::Duration(0.0);
        duration = ros::Time::now() - it->second;

        Site obj_under_vcs = tfMapToBaseLink(ivlocpos, it->first);

        ROS_INFO_STREAM("[fyn]:new_obj is:"
                        << "(" << obj_under_vcs.x << "," << obj_under_vcs.y << ")");

        if (std::fabs(duration.toSec()) > 5.0)
        {
            ROS_WARN_STREAM("[fyn]:clear (" << it->first.x << "," << it->first.y << ")");
            blind_area.erase(it);
        }
        else
        {
            geometry_msgs::Point32 tt;
            tt.x = obj_under_vcs.x;
            tt.y = obj_under_vcs.y;
            tt.z = 0.0;
            if (!collid_obstacles[tt.x].count(tt))
            {
                collid_obstacles[tt.x].insert(tt);
            }
            if (!emergencyObsTree[tt.x].count(tt))
            {
                emergencyObsTree[tt.x].insert(tt);
            }
        }
    }
}

void omegapathplanner::GetNoAvoidArea(std::string &routemap)
{
    no_avoid_area_.clear();

    std::string input_path;
    input_path = routemap + "/" + "no_avoid_area";

    std::ifstream in_file(input_path);
    std::string tmp_line;

    while (getline(in_file, tmp_line))
    {
        std::vector<double> value_buf;
        std::stringstream ss;
        ss << tmp_line;

        std::string tmp_value_str;

        while (getline(ss, tmp_value_str, ','))
        {
            std::stringstream ss_value;
            double tmp_value;
            ss_value << tmp_value_str;
            ss_value >> tmp_value;
            value_buf.push_back(tmp_value);
        }

        ivmap::ivmapmsglocpos lt_pos, rd_pos;
        lt_pos.xg = value_buf[0];
        lt_pos.yg = value_buf[1];
        rd_pos.xg = value_buf[2];
        rd_pos.yg = value_buf[3];

        auto tmp_area = std::make_pair(lt_pos, rd_pos);
        no_avoid_area_.push_back(tmp_area);
    }
}

void omegapathplanner::OptionAvoid()
{
    area_avoid_ = true;

    for (auto &area : no_avoid_area_)
    {
        ivmap::ivmapmsglocpos left_top_pos;
        ivmap::ivmapmsglocpos right_down_pos;
        left_top_pos = area.first;
        right_down_pos = area.second;

        if (left_top_pos.xg < ivlocpos.xg && ivlocpos.xg < right_down_pos.xg &&
            left_top_pos.yg < ivlocpos.yg && ivlocpos.yg < right_down_pos.yg)
        {
            area_avoid_ = false;
        }
    }
}
