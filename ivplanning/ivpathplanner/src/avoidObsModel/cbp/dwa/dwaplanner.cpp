#include "dwaplanner.h"

dwaplanner::dwaplanner(ros::NodeHandle nh)
: stepsize(0.15)
, steplength(0.6)
, controlCycle(0.9)
, yawtolerance(9.0)
, disttolerance(0.6)
, REVERSE(false)
, reachedGoal(false)
, gicsCellSize(0.05)
{
    visdwa_marker = std::unique_ptr<visualize_marker>(new visualize_marker("/dwa"));

    for (int i = 0; i <= 20; ++i)
    {
        samplenum.emplace(i, i);
    }

    near_goal = true;

    initialstatus.x = 0.0;
    initialstatus.y = 0.0;
    initialstatus.heading = functions::deg2Rad(0.0);
    initialstatus.linear_velocity = 0.0;
    initialstatus.angular_velocity = 0.0;

    goalstatus.x = 12.0;
    goalstatus.y = 12.0;
    goalstatus.heading = functions::deg2Rad(0.0);
    goalstatus.angular_velocity = functions::deg2Rad(0.0);
    goalstatus.linear_velocity = 0.0;

    //This is the recommended centrifugal_acceleration
    dyn.centrifugal_acceleration = 1.0;
    dyn.max_linear_velocity = 2.0;
    dyn.max_linear_acceleration = 1.0;
    dyn.min_turn_radius = functions::radius;

    nh.param ( "basemap", mapPath, mapPath );
    nh.param ( "gicscellsize", gicsCellSize, gicsCellSize );
    nh.param ( "iabasemaptllon", iabasemapTL.lon, iabasemapTL.lon );
    nh.param ( "iabasemaptllat", iabasemapTL.lat, iabasemapTL.lat );
    nh.param ( "iabasemapbrlon", iabasemapBR.lon, iabasemapBR.lon );
    nh.param ( "iabasemapbrlat", iabasemapBR.lat, iabasemapBR.lat );

    mapTool.initParam ( mapPath, gicsCellSize, iabasemapTL, iabasemapBR);
}

dwaplanner::dwaplanner()
{
}

dwaplanner::~dwaplanner() {}

int dwaplanner::collisiondetection(std::vector<geometry_msgs::Point32> ego, sorted_tree &obspts, sVehicleElem vehiclemodel)
{
    geometry_msgs::Point32 p, q;
    geometry_msgs::Polygon poly, littlepoly;
    if (obspts.size() == 0)
    {
        return 0;
    }
    for (auto tp : ego)
    {
        std::tie(p, q) = functions::carmodel(tp, poly, vehiclemodel.frontoverhang, vehiclemodel.backoverhang, vehiclemodel.halfwheeltrack);

        sPointOfVCS modelPt;
        modelPt.x = tp.x;
        modelPt.y = tp.y;

        #if USE_FREE_SPACE
        if (false == mapTool.isInFreeSpace (egoPos, modelPt))
        {
            return 1;
        }
        functions::carmodel(tp, littlepoly, 0.9, 0.3, 0.3);
        for (auto a : littlepoly.points)
        {
            modelPt.x = a.x;
            modelPt.y = a.y;
            if (false == mapTool.isInFreeSpace (egoPos, modelPt))
            {
                return 1;
            }
        }
        #endif


        auto ha = obspts.lower_bound(p.x);
        if (!obspts.count(ha->first) || ha->first > q.x)
        {
            continue;
        }
        else
        {
            for (; ha->first <= q.x && ha != obspts.end(); ++ha)
            {
                auto hi = obspts[ha->first].lower_bound(p);
                if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
                {
                    continue;
                }
                else
                {
                    for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
                    {
                        if (functions::IsInsideFootprint(*hi, poly))
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int dwaplanner::collisiondetection(ivpathplanner::path ego, sorted_tree &obspts, sVehicleElem vehiclemodel)
{
    geometry_msgs::Point32 p, q, tp;
    geometry_msgs::Polygon poly;

    for (int i = 0; i < ego.points.size(); ++i)
    {
        tp.x = ego.points.at(i).x;
        tp.y = ego.points.at(i).y;
        tp.z = ego.points.at(i).angle * M_PI / 180.0;

        std::tie(p, q) = functions::carmodel(tp, poly, vehiclemodel.frontoverhang, vehiclemodel.backoverhang, vehiclemodel.halfwheeltrack);     

        auto ha = obspts.lower_bound(p.x);
        if (!obspts.count(ha->first) || ha->first > q.x)
        {
            continue;
        }
        else
        {
            for (; ha->first <= q.x && ha != obspts.end(); ++ha)
            {
                auto hi = obspts[ha->first].lower_bound(p);
                if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
                {
                    continue;
                }
                else
                {
                    for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
                    {
                        if (functions::IsInsideFootprint(*hi, poly))
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int dwaplanner::collisiondetection(std::vector<sVehicleStatus> ego, sorted_tree &obspts, sVehicleElem vehiclemodel)
{
    geometry_msgs::Point32 p, q, tp;
    geometry_msgs::Polygon poly, littlepoly;

    if (obspts.empty() || ego.empty())
    {
        return 0;
    }
    for (auto tt : ego)
    {
        tp.x = tt.x;
        tp.y = tt.y;
        tp.z = tt.heading;

        std::tie(p, q) = functions::carmodel(tp, poly, vehiclemodel.frontoverhang, vehiclemodel.backoverhang, vehiclemodel.halfwheeltrack);     

        sPointOfVCS modelPt;
        modelPt.x = tp.x;
        modelPt.y = tp.y;

        #if USE_FREE_SPACE
        if (false == mapTool.isInFreeSpace(egoPos, modelPt))
        {
            return 1;
        }

        functions::carmodel(tp, littlepoly, 0.9, 0.3, 0.3);
        for (auto a : littlepoly.points)
        {
            modelPt.x = a.x;
            modelPt.y = a.y;
            if (false == mapTool.isInFreeSpace(egoPos, modelPt))
            {
                return 1;
            }
        }
        #endif

        auto ha = obspts.lower_bound(p.x);

        if (!obspts.count(ha->first) || ha->first > q.x)
        {
            continue;
        }
        else
        {
            for (; ha->first <= q.x && ha != obspts.end(); ++ha)
            {
                auto hi = obspts[ha->first].lower_bound(p);
                if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
                {
                    continue;
                }
                else
                {
                    for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
                    {
                        if (functions::IsInsideFootprint(*hi, poly))
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }
    return 0;
}

int dwaplanner::collisiondetection(std::vector<sVehicleStatus> ego, sorted_tree &obspts,
                                   sVehicleStatus goalvs, bool& reachedGoal, sVehicleElem vehiclemodel)
{
    geometry_msgs::Point32 p, q, tp;
    geometry_msgs::Polygon poly, littlepoly;

    if (obspts.size() == 0 || ego.empty())
    {
        return 0;
    }
    for (auto tt : ego)
    {
        reachedGoal = reachgoal(tt, goalvs);
        if (true == reachedGoal)
        {
            return 0;
        }    

        tp.x = tt.x;
        tp.y = tt.y;
        tp.z = tt.heading;

        std::tie(p, q) = functions::carmodel(tp, poly, vehiclemodel.frontoverhang, vehiclemodel.backoverhang, vehiclemodel.halfwheeltrack);     

        sPointOfVCS modelPt;
        modelPt.x = tp.x;
        modelPt.y = tp.y;

        #if USE_FREE_SPACE
        if (false == mapTool.isInFreeSpace(egoPos, modelPt))
        {
            return 1;
        }

        functions::carmodel(tp, littlepoly, 0.9, 0.3, 0.3);
        for (auto a : littlepoly.points)
        {
            modelPt.x = a.x;
            modelPt.y = a.y;
            if (false == mapTool.isInFreeSpace(egoPos, modelPt))
            {
                return 1;
            }
        }
        #endif

        auto ha = obspts.lower_bound(p.x);

        if (!obspts.count(ha->first) || ha->first > q.x)
        {
            continue;
        }
        else
        {
            for (; ha->first <= q.x && ha != obspts.end(); ++ha)
            {
                auto hi = obspts[ha->first].lower_bound(p);
                if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
                {
                    continue;
                }
                else
                {
                    for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
                    {
                        if (functions::IsInsideFootprint(*hi, poly))
                        {
                            return 1;
                        }
                    }
                }
            }
        }
    }

    /* For the emergency braking! */
    /*So as to avoid the near obstacles!!!*/

    sVehicleStatus brakingvs = ego.back();

    for (float bb = 1.0; bb <= 5.0; bb += 1.0)
    {
        tp.x = brakingvs.x + bb * cos(brakingvs.heading);
        tp.y = brakingvs.y + bb * sin(brakingvs.heading);
        tp.z = brakingvs.heading;

        sPointOfVCS modelPt;
        modelPt.x = tp.x;
        modelPt.y = tp.y;

        #if 1

        //space exploration process
        if (false == mapTool.isInFreeSpace(egoPos, modelPt))
        {
            return 2;
        }

        functions::carmodel(tp, littlepoly, 0.9, 0.3, 0.3);
        for (auto a : littlepoly.points)
        {
            modelPt.x = a.x;
            modelPt.y = a.y;
            if (false == mapTool.isInFreeSpace(egoPos, modelPt))
            {
                return 2;
            }
        }

        #endif

        std::tie(p, q) = functions::carmodel(tp, poly, vehiclemodel.frontoverhang, vehiclemodel.backoverhang, vehiclemodel.halfwheeltrack);

        auto ha = obspts.lower_bound(p.x);

        if (obspts.count(ha->first) && ha->first <= q.x)
        {
            for (; ha->first <= q.x && ha != obspts.end(); ++ha)
            {
                auto hi = obspts[ha->first].lower_bound(p);
                if (!obspts[ha->first].count(*hi) || (*hi).y > q.y)
                {
                    continue;
                }
                else
                {
                    for (; (*hi).y <= q.y && hi != obspts[ha->first].end(); ++hi)
                    {
                        if (functions::IsInsideFootprint(*hi, poly))
                        {
                            return 3;
                        }
                    }
                }
            }
        }
    }

   /**********************************************/

    return 0;
}


std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>>
dwaplanner::GenerateTrajectory(sVehicleStatus currentvs, sKinematic sk)
{
    trajectorys.clear();

    sVehicleStatus subsequent;

    // linear_velocity, angular_velocity, and G cost
    std::vector<std::tuple<float, float, float>> speedarray;
    std::vector<std::tuple<sVehicleStatus, std::vector<sVehicleStatus>, float>> rtn;

    int angular_num = 4;
    int speed_num = 1;
    float angular_sample = 0.0;
    float speed_sample = 0.0;
    float acceleration_coefficient = 0.3;
    float linear_velocity, angular_velocity;

    subsequent = currentvs;

    /*    the reversing mode sampling     */
    if (REVERSE)
    {
        controlCycle = 2.0;
        std::vector<float> vec_speed(3);
        vec_speed.at(0) = -0.5;
        vec_speed.at(1) = -0.8;
        vec_speed.at(2) = -1.0;

        for (int i = 0; i < vec_speed.size(); ++i)
        {
            linear_velocity = vec_speed.at(i);

            sk.max_angular_velocity = fabs(linear_velocity) / sk.min_turn_radius;
            if (fabs(linear_velocity * sk.max_angular_velocity) > (sk.centrifugal_acceleration + 1e-3))
            {
                sk.max_angular_velocity = sk.centrifugal_acceleration / fabs(linear_velocity);
            }
            if (fabs(linear_velocity) <= 0.5)
            {
                angular_num = samplenum[3];
            }
            else
            {
                angular_num = samplenum[4]; 
            }
            angular_sample = sk.max_angular_velocity / angular_num;
            for (int j = -angular_num; j <= angular_num; ++j)
            {
                angular_velocity = currentvs.angular_velocity + angular_sample * j;
                if (fabs(angular_velocity) < 1e-2)
                {
                    angular_velocity = 0.0;
                }
                if (fabs(angular_velocity) > (sk.max_angular_velocity + 1e-3))
                {
                    continue;
                }
                subsequent.linear_velocity = linear_velocity;
                subsequent.angular_velocity = angular_velocity;
                speedarray.emplace_back(
                std::forward_as_tuple(
                linear_velocity, angular_velocity, gValUpdate(currentvs, subsequent)
                ));
            }
        }
    }
    /*    the normal mode sampling    */
    else
    {
        controlCycle = 0.9;
        speed_num = samplenum[1];
        speed_sample = acceleration_coefficient * sk.max_linear_acceleration * controlCycle / speed_num;
        for (int i = speed_num; i >= -speed_num; i += -1)
        {
            linear_velocity = currentvs.linear_velocity + speed_sample * i;
            if (linear_velocity < (initialstatus.linear_velocity - 1e-2) || linear_velocity > (goalstatus.linear_velocity + 1e-2))
            {
                continue;
            }
            // cout<<"linear_velocity: "<<linear_velocity<<endl;
            sk.max_angular_velocity = fabs(linear_velocity) / sk.min_turn_radius;
            if (fabs(linear_velocity * sk.max_angular_velocity) > (sk.centrifugal_acceleration + 1e-3))
            {
                sk.max_angular_velocity = sk.centrifugal_acceleration / linear_velocity;
            }
            if (linear_velocity <= 0.5)
            {
                angular_num = samplenum[2];
            }
            else if (linear_velocity <= 2.0)
            {
                angular_num = samplenum[3];
            }
            else
            {
                angular_num = samplenum[4];
            }
    
            angular_sample = sk.max_angular_velocity / angular_num;
            for (int j = -angular_num; j <= angular_num; ++j)
            {
                /*The left side has higher priority*/
                angular_velocity = currentvs.angular_velocity - angular_sample * j;
                if (fabs(angular_velocity) < 1e-2)
                {
                    angular_velocity = 0.0;
                }
                if (fabs(angular_velocity) > (sk.max_angular_velocity + 1e-3))
                {
                    continue;
                }
                subsequent.linear_velocity = linear_velocity;
                subsequent.angular_velocity = angular_velocity;
                speedarray.emplace_back(
                std::forward_as_tuple(
                linear_velocity, angular_velocity, gValUpdate(currentvs, subsequent)
                ));
            }
        }
    }

    /**The trajectory samping process**/
    for (auto spd : speedarray)
    {
        std::vector<sVehicleStatus> path;
        sVehicleStatus tempvs = currentvs;
        path.push_back(tempvs);

        tempvs.linear_velocity = std::get<0>(spd);
        tempvs.angular_velocity = std::get<1>(spd);

        int diffnum = fabs(static_cast<int>(tempvs.linear_velocity * controlCycle / stepsize));
        float timestep = fabs(stepsize / tempvs.linear_velocity);

        for (int a = 0; a < diffnum; ++a)
        {
            // Vehicle state sampling
            /** A little bug causes great pain !!! **/
            if (fabs(tempvs.angular_velocity) < 1e-2)
            {
                tempvs.x += tempvs.linear_velocity * timestep * cos(tempvs.heading);
                tempvs.y += tempvs.linear_velocity * timestep * sin(tempvs.heading);
            }
            else
            {
                //@brief refer to the <<Probabilistic robotics>>, table 5.3, 5.9
                tempvs.heading += tempvs.angular_velocity * timestep;
                tempvs.x += (tempvs.linear_velocity / tempvs.angular_velocity) *
                (sin(tempvs.heading + tempvs.angular_velocity * timestep) - sin(tempvs.heading));
                tempvs.y += (tempvs.linear_velocity / tempvs.angular_velocity) *
                (-cos(tempvs.heading + tempvs.angular_velocity * timestep) + cos(tempvs.heading));
            }
            path.push_back(tempvs);
        }
        trajectorys.push_back(path);
        rtn.emplace_back(std::forward_as_tuple(path.back(), path, std::get<2>(spd)));
    }

    return rtn;
}

float dwaplanner::gValUpdate(sVehicleStatus cur, sVehicleStatus next)
{
    float gval = 0.0;
    gval = fabs(next.linear_velocity) * controlCycle;
    return gval;
}

//This H-value updating function bind the dubins_curve and the reeds_shepp as hValUpdate!!!
float dwaplanner::hValUpdate(sVehicleStatus cur, sVehicleStatus goal)
{
    float hvalue = 0.0;
    float circle_radius = functions::radius;
    geometry_msgs::Point32 p1, p2;
    std::vector<geometry_msgs::Point32> arr;
    p1.x = cur.x;
    p1.y = cur.y;
    p1.z = cur.heading;
    p2.x = goal.x;
    p2.y = goal.y;
    p2.z = goal.heading;
    double q0[3] = {cur.x, cur.y, cur.heading};
    double q1[3] = {goal.x, goal.y, goal.heading};
    auto cb = [](double q[3], double x, void *user_data) -> int {
        return 1;
    };

    if (true == REVERSE)
    {
        ReedsSheppStateSpace reeds_shepp(circle_radius);
        hvalue = reeds_shepp.distance(q0, q1);
    }
    else
    {
        dubins_curve::DubinsPath dubins_path;
        dubins_curve::dubins_init(q0, q1, circle_radius, &dubins_path);
        dubins_curve::dubins_path_sample_many(&dubins_path, cb, 0.2, nullptr);
        hvalue = dubins_curve::dubins_path_length(&dubins_path);     
    }

    return hvalue;
}

//The kernel function of this god-like algorithm
sVehicleStatus dwaplanner::dynamicWindowApproach(sVehicleStatus start, sVehicleStatus goal, unordered_t<sVehicleStatus> &came_from,
unordered_t<float> &cost_so_far, unordered_t<std::vector<sVehicleStatus>> &pathtable, sorted_tree sortedobjs)
    {
        reachedGoal = false;
        int remark = 0;
        dyn.max_linear_velocity = goal.linear_velocity;

        sVehicleStatus current;
        functions::PriorityNode<sVehicleStatus, float> frontier;
        frontier.elements.emplace(start, 0.0);
        came_from[start] = start;
        cost_so_far[start] = 0.0;

        while (!frontier.elements.empty())
        {
            current = frontier.elements.top().first;
            frontier.elements.pop();
            //This algorithm is like holy shit, and perfectly wonderful!!!
            sVehicleElem vehiclemodel;
            vehiclemodel.frontoverhang = 1.5;
            vehiclemodel.backoverhang = 0.3;
            vehiclemodel.halfwheeltrack = 0.70;
            int collisionflag = collisiondetection(pathtable[current], sortedobjs, goal, reachedGoal, vehiclemodel);
            if (1 == collisionflag)
            {
                continue;
            }
            if (remark >= 200 || true == reachedGoal)
            {
                break;
            }
            
            for (auto next : GenerateTrajectory(current, dyn))
            {
                //Compare the algorithm efficiency!!!
                auto nextpath = std::get<1>(next);
                sVehicleStatus nextvs = std::get<0>(next);
                float new_cost = cost_so_far[current] + std::get<2>(next);
                new_cost = 0.0;

                // Only if the successor is not in the close table
                if (!came_from.count(nextvs))
                {
                    // If the successor is not in the open table, If the successor is in the open table
                    if (!cost_so_far.count(nextvs) || ((new_cost + 1e-1) < cost_so_far[nextvs]))
                    {
                        float temph = hValUpdate(nextvs, goal);
                        /*Explore the unknown world and step back*/

                        if (0 != collisionflag)
                        {
                            temph += (1 + 0.1 * collisionflag) * std::get<2>(next);
                        }

                        float priority = new_cost + temph;
                        frontier.elements.emplace(nextvs, priority);
                        cost_so_far[nextvs] = new_cost;
                        came_from[nextvs] = current;
                        pathtable[nextvs] = nextpath;
                    }
                }
            }
            remark++;
        }
        return current;
    }

void dwaplanner::obstaclemodify(ivmap::ivmapmsgstcell objs, sorted_tree &nearestobs, sorted_tree &rawobs, sorted_tree &dilatedobs)
{
    geometry_msgs::Point32 tt, tp;
    tp.x = 0.0;
    tp.y = 0.0;
    tp.z = 0.0;
    geometry_msgs::Polygon poly;
    functions::carmodel(tp, poly, 1.2, 0.3, 0.8);

    for (auto b : objs.cell)
    {
        tt.x = b.xc * 0.1;
        tt.y = b.yc * 0.1;
        float dis = std::hypot(tt.x, tt.y);
        if (dis > 12.0)
        {
            continue;
        }
        else
        {
            /*This is used for emergency stop, reversing driving and slowing down*/

            if (true == functions::IsInsideFootprint(tt, poly))
            {
                continue;
            }

            if (!nearestobs[tt.x].count(tt))
            {
                nearestobs[tt.x].insert(tt);
            }

            if (!rawobs[tt.x].count(tt))
            {
                rawobs[tt.x].insert(tt);
            }
            if (!dilatedobs[tt.x].count(tt))
            {
                dilatedobs[tt.x].insert(tt);
            }
            if (dis > 3.0)
            {
                for (int i = -1; i <= 1; i += 2)
                {
                    for (int j = -1; j <= 1; j += 2)
                    {
                        tp.x = tt.x + 0.1 * i;
                        tp.y = tt.y + 0.1 * j;
                        if (!dilatedobs[tp.x].count(tp))
                        {
                            dilatedobs[tp.x].insert(tp);
                        }
                    }
                }
            }
        }
    }
    return;
}

void dwaplanner::obstaclemodify(ivpredict::ivmsgpredict objs, sorted_tree &nearestobs,
                                sorted_tree &rawobs, sorted_tree &dilatedobs, float carspeed)
{
    float dis = 0.0;
    geometry_msgs::Point32 tt, tp;
    tp.x = 0.0;
    tp.y = 0.0;
    tp.z = 0.0;
    geometry_msgs::Polygon poly;
    functions::carmodel(tp, poly, 1.2, 0.3, 0.8);
    sorted_tree().swap(dynamic_obs);
    for (auto a : objs.objects)
    {
        /*This is the non-static case*/
        if (-1 != a.state)
        {
            for (auto b : a.cell)
            {
                tt.x = b.xc * 0.1;
                tt.y = b.yc * 0.1;
                dis = std::hypot(tt.x, tt.y);
                if (dis < 12.0)
                {
                    if (!nearestobs[tt.x].count(tt))
                    {
                        nearestobs[tt.x].insert(tt);
                    }
                }
            }
        }
        /*This is the static case*/
        else
        {
            for (auto b : a.cell)
            {
                tt.x = b.xc * 0.1;
                tt.y = b.yc * 0.1;
                dis = std::hypot(tt.x, tt.y);
                if (dis > 12.0)
                {
                    continue;
                }
                else
                {
                    /*Push the near obstacles*/

                    if (true == functions::IsInsideFootprint(tt, poly))
                    {
                        continue;
                    }

                    if (!nearestobs[tt.x].count(tt))
                    {
                        nearestobs[tt.x].insert(tt);
                    }

                    if (!rawobs[tt.x].count(tt))
                    {
                        rawobs[tt.x].insert(tt);
                    }
                    if (!dilatedobs[tt.x].count(tt))
                    {
                        dilatedobs[tt.x].insert(tt);
                    }

                    if (dis > 3.0)
                    {
                        if (a.width > 1.0)
                        {
                            for (int i = -2; i <= 2; i += 2)
                            {
                                for (int j = -1; j <= 1; j += 2)
                                {
                                    tp.x = tt.x + 0.1 * i;
                                    tp.y = tt.y + 0.1 * j;
                                    if (!dilatedobs[tp.x].count(tp))
                                    {
                                        dilatedobs[tp.x].insert(tp);
                                    }
                                }
                            }
                        }
                    }
                }
            }           
        }
    }
    return;
}

int dwaplanner::TurningBehavior(ivmap::ivmapmsglocpos ivlocpos, ivpathplanner::path current,
                                ivpathplanner::path& stoppingPoint)
{
    stoppingPoint.points.clear();
    stoppingPoint.stopflag = false;
    int turning = 0;
    // if (!current.points.empty())
    int num = current.points.size();
    if (num > 10 && fabs(current.points.at(0.5 * num).angle - current.points.back().angle) < 10.0)
    {

        if (fabs(current.points.back().angle - 90.0) < 30.0 )
        {
            //left
            turning = -1;
        }
        else if (fabs(current.points.back().angle + 90.0) < 30.0)
        {
            //right
            turning = 1;
        }          

    }
    return turning;
    if (0 == turning)
    {
        return turning;
    }
    for (auto iter = current.points.begin(); iter != current.points.end(); ++iter)
    {
        if (fabs(iter->angle) < 30.0)
        {
            iter = current.points.erase(iter);
        }
        else
        {
            iter++;
            stoppingPoint.points.push_back(*iter);
            break;
            //Get the resting path;
        }
    }

    sVehicleElem vehiclemodel;
    vehiclemodel.frontoverhang = 1.5;
    vehiclemodel.backoverhang = 0.3;
    vehiclemodel.halfwheeltrack = 0.50;
    int collisionflag = collisiondetection(current, dynamic_obs, vehiclemodel);
    stoppingPoint.stopflag = static_cast<bool>(collisionflag);
    return turning;
}

/**
* @brief
* @param
*/
int dwaplanner::reachgoal(sVehicleStatus currentvs, sVehicleStatus goalvs)
{
    float dis, theta;
    dis = std::hypot(currentvs.x - goalvs.x, currentvs.y - goalvs.y);
    if (dis > disttolerance)
    {
        return 0;
    }
    theta = functions::rad2Deg(fabs(currentvs.heading - goalvs.heading));
    if (theta > yawtolerance)
    {
        return 0;
    }
    return 1;
}

std::vector<sVehicleStatus>
dwaplanner::reconstructPath(sVehicleStatus start, sVehicleStatus goal,
unordered_t<sVehicleStatus> &came_from, sVehicleStatus last)
{
    std::vector<sVehicleStatus> path;
    sVehicleStatus current = last;
    path.push_back(current);
    while (!(current == start))
    {
        current = came_from[current];
        path.push_back(current);
    }
    //   path.push_back(start);
    std::reverse(path.begin(), path.end());
    //   path.push_back(goal);
    return path;
}

//Interface to collision avoiding module of path planning
ivpathplanner::path dwaplanner::pathinterface(ivmap::ivmapmsglocpos ivlocpos,
ivpathplanner::path current, ivpathplanner::path rawpath, sorted_tree sortedobjs)
{
    if (sortedobjs.empty())
    {
        return current;
    }
    egoPos.xg = ivlocpos.xg;
    egoPos.yg = ivlocpos.yg;
    egoPos.angle = ivlocpos.angle;

    std::vector<int> roadSegId;
    ivpathplanner::path ivpath;

    float dis = 0.0;
    float totaldis = 0.0;
    if (rawpath.points.size() > 10)
    {
        for (int i = 0; i < rawpath.points.size() - 2; ++i)
        {
            float tempdis = 0.0;
            tempdis = std::hypot(rawpath.points.at(i + 1).x - rawpath.points.at(i).x, rawpath.points.at(i + 1).y - rawpath.points.at(i).y);
            totaldis += tempdis;
            if (totaldis < 12.0)
            {
                continue;
            }
            dis += tempdis;
            if (dis > 1.0)
            {
                roadSegId.push_back(i);
                dis = 0.0;
            }
        }
        initialstatus.x = 0.0;
        initialstatus.y = 0.0;
        initialstatus.heading = 0.0;
        initialstatus.linear_velocity = 0.4;
        initialstatus.angular_velocity = 0.0;
    }

    near_goal = false;
    if (roadSegId.empty())
    {
        near_goal = true;
    }

    const int chip_num = 4;
    int chip_size = roadSegId.size() / (chip_num - 1);
    if (chip_size <= 1)
    {
        chip_size = 2;
    }

    std::vector<int> point_id;
    int chip_pos = 0;
    while(chip_pos < roadSegId.size())
    {
        point_id.push_back(roadSegId.at(chip_pos));
        chip_pos += chip_size;
    }

    for (auto cc : point_id)
    {
        goalstatus.x = rawpath.points.at(cc).x;
        goalstatus.y = rawpath.points.at(cc).y;
        goalstatus.heading = rawpath.points.at(cc).angle * M_PI / 180.0;

        goalstatus.linear_velocity = 2.0;
        goalstatus.angular_velocity = 0.0;

        std::vector<sVehicleStatus> rtnpath;
        sVehicleStatus lst;
        unordered_t<sVehicleStatus> came_from;
        unordered_t<float> cost_so_far;
        unordered_t<std::vector<sVehicleStatus> > pathtable;
        lst = dynamicWindowApproach(initialstatus, goalstatus, came_from, cost_so_far, pathtable, sortedobjs);
        rtnpath = reconstructPath(initialstatus, goalstatus, came_from, lst);

        if (true == reachedGoal)
        {
            for (int i = 0; i < rtnpath.size(); ++i)
            {
                for (auto val : pathtable[rtnpath.at(i)])
                {
                    ivpathplanner::pathpoint temp;
                    temp.x = val.x;
                    temp.y = val.y;
                    temp.angle = val.heading * 180.0 / M_PI;
                    temp.velocity = val.linear_velocity;
                    temp.velocity = -88.0;
                    ivpath.points.push_back(temp);
                }
            }
            return ivpath;
        }
        else
        {
            tf::Quaternion tfquater;
            geometry_msgs::Point32 st, gt, tmp;
            geometry_msgs::Pose start_pose, end_pose;
            std::vector<geometry_msgs::Point32> link_curve;
            auto backpt = pathtable[rtnpath.back()].back();
            if (std::hypot(backpt.x - goalstatus.x, backpt.y - goalstatus.y) > 2.0)
            {
                continue;
            }
            st.x = backpt.x;
            st.y = backpt.y;
            st.z = backpt.heading;
            start_pose.position.x = backpt.x;
            start_pose.position.y = backpt.y;
            start_pose.orientation = tf::createQuaternionMsgFromYaw(backpt.heading);
            gt.x = goalstatus.x;
            gt.y = goalstatus.y;
            gt.z = goalstatus.heading;
            end_pose.position.x = goalstatus.x;
            end_pose.position.y = goalstatus.y;
            end_pose.orientation = tf::createQuaternionMsgFromYaw(goalstatus.heading);

            std::vector<geometry_msgs::Pose> raw_path;
            raw_path = generatecurve::generateHermiteCurveForROS(start_pose, end_pose, 2.0);
            for (auto a : raw_path)
            {
                tmp.x = a.position.x;
                tmp.y = a.position.y;
                tf::quaternionMsgToTF(a.orientation, tfquater);
                tmp.z = tf::getYaw(tfquater);
                link_curve.push_back(tmp);
            }

            sVehicleElem vehiclemodel;
            vehiclemodel.frontoverhang = 1.5;
            vehiclemodel.backoverhang = 0.3;
            vehiclemodel.halfwheeltrack = 0.7;
            bool collidflag = collisiondetection(link_curve, sortedobjs, vehiclemodel);
            if (false == collidflag)
            {
                for (int i = 0; i < rtnpath.size(); ++i)
                {
                    for (auto val : pathtable[rtnpath.at(i)])
                    {
                        ivpathplanner::pathpoint temp;
                        temp.x = val.x;
                        temp.y = val.y;
                        temp.angle = val.heading * 180.0 / M_PI;
                        temp.velocity = val.linear_velocity;
                        temp.velocity = -88.0;
                        ivpath.points.push_back(temp);
                    }
                }
                for (auto val : link_curve)
                {
                    ivpathplanner::pathpoint temp;
                    temp.x = val.x;
                    temp.y = val.y;
                    temp.angle = val.z * 180.0 / M_PI;
                    temp.velocity = -88.0;
                    ivpath.points.push_back(temp);
                }
                return ivpath;
            }
        }
    }
    return ivpath;
}

//Interface to collision avoiding module of path planning
ivpathplanner::path dwaplanner::ReverseInterface(ivmap::ivmapmsglocpos ivlocpos,
ivpathplanner::path current, sorted_tree sortedobjs)
{
    ivpathplanner::path ivpath;
    if (sortedobjs.empty())
    {
        return current;
    }

    egoPos.xg = ivlocpos.xg;
    egoPos.yg = ivlocpos.yg;
    egoPos.angle = ivlocpos.angle;

    REVERSE = true;
    initialstatus.x = 0.0;
    initialstatus.y = 0.0;
    initialstatus.heading = 0.0;
    initialstatus.linear_velocity = 0.4;
    initialstatus.angular_velocity = 0.0;

    ivpathplanner::pathpoint comparison_point;
    if (!current.points.empty())
    {
        comparison_point = current.points.back();
    }

    unordered_t<std::vector<sVehicleStatus>> pathtable;
    functions::PriorityNode<sVehicleStatus, float> frontier;

    std::vector<sVehicleStatus> normalize_status;
    std::vector<float> normalize_orientation;
    std::vector<float> normalize_distance;
    float total_orientation = 0.0;
    float total_distance = 0.0;
    
    for (auto next : GenerateTrajectory(initialstatus, dyn))
    {
        auto nextpath = std::get<1>(next);
        auto nextvs = std::get<0>(next);

        sVehicleElem vehiclemodel;
        vehiclemodel.frontoverhang = 1.0;
        vehiclemodel.backoverhang = 0.3;
        vehiclemodel.halfwheeltrack = 0.4;
        int collidflag = collisiondetection(nextpath, sortedobjs, vehiclemodel);
        if (1 == collidflag)
        {
            continue;
        }

        /*The less theta variation has higher priority, and normalized*/ 
        float priority_orientation = fabs(nextvs.heading * 180.0 / M_PI - comparison_point.angle);
        float priority_distance = fabs(nextvs.x * sin(comparison_point.angle) - nextvs.y * cos(comparison_point.angle) - 
                                       comparison_point.x * sin(comparison_point.angle) + comparison_point.y * cos(comparison_point.angle));
        total_orientation += priority_orientation;
        total_distance += priority_distance;
        normalize_status.emplace_back(nextvs);
        normalize_orientation.emplace_back(priority_orientation);
        normalize_distance.emplace_back(priority_distance);
        // frontier.elements.emplace(nextvs, priority);

        pathtable[nextvs] = nextpath;
    }

    /*Just in case of deviding the zero number*/
    total_orientation += 1e-2;
    total_distance += 1e-1;

    if (!normalize_status.empty()){
        for (int i = 0; i < normalize_status.size(); ++i){
            float priority = normalize_orientation.at(i) / total_orientation + normalize_distance.at(i) / total_distance;
            frontier.elements.emplace(normalize_status.at(i), priority);
        }
    }


    if (!frontier.elements.empty()){
        sVehicleStatus optimum = frontier.elements.top().first;
        frontier.elements.pop();
        for (auto val : pathtable[optimum]){
            ivpathplanner::pathpoint temp;
            temp.x = val.x;
            temp.y = val.y;
            temp.angle = val.heading * 180.0 / M_PI;
            temp.velocity = val.linear_velocity;
            temp.velocity = 0.3;
            ivpath.points.push_back(temp);
        }
    }
    REVERSE = false;
    return ivpath;
}

void dwaplanner::testModule()
{
    return;
}

