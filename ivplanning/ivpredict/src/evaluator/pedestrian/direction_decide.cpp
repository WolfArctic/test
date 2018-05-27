/**************************************************************
Copyright (C) 2015-2020, idriverplus(Beijing ZhiXingZhe Inc.)
NodeName: ivpredict
FileName: direction_decide.cpp
Description: 
1. main function

History:
<author>    <time>      <version>    <description>

************************************************************/
#include "direction_decide.h"

direction_decide::direction_decide(){

}
direction_decide::~direction_decide(){

}
int direction_decide::UpdateState(ivpredict::objbuffer *pObj, ivpredict::predictobj* self_predict, ivpredict::predictobj* answer_predict){
	int frequency = 2;
	int count_time = 2;
	int state_now = pObj->state;


	bool buffer_update = pObj->update;
	int temp_buffer_size = pObj->positions.size();
	if (buffer_update)
	{
		int getdis_succ = GetDis(pObj,self_predict);
		int update_succ = UpdateStates(pObj,self_predict);
	}
	// cout << " lihui obj:" << pObj->id << " state:" << pObj->state << " v_line:" << pObj->positions[temp_buffer_size-1].vdis << endl;
	if (pObj->positions[temp_buffer_size-1].xmean < -2 && abs(pObj->positions[temp_buffer_size-1].ymean) < 1.5)
	{
		state_now = 0;
	}
	switch (state_now)
	{
		//stop
		case -1:{
			int predict_succ = Stop(pObj,answer_predict);
			// float time_now = ros::Time::now().toSec() * 1000;
			if (ToParallel(pObj))
			{
				pObj->stay_count++;
				if (pObj->stay_count > count_time)
				{
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 1;
				}
			}
			else if (To45(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 3;
				// }
			}
			else if (ToCross(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 2;
				// }
			}
			break;
		}
		//init
		case 0:{
			int predict_succ = NoPredict(pObj,answer_predict);
			// float time_now = ros::Time::now().toSec() * 1000;
			if (To45(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 3;
				// }
			}
			else if (ToCross(pObj) && (temp_buffer_size > frequency))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 2;
				// }
			}
			else if (temp_buffer_size > frequency)
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 1;
				// }
			}
			// else if (ToStop(pObj) && (temp_buffer_size > frequency))
			// {
			// 	// pObj->stay_count++;
			// 	// if (pObj->stay_count > count_time)
			// 	// {
			// 		pObj->stay_count = 0;
			// 		// pObj->change_time = ros::Time::now().toSec() * 1000;
			// 		pObj->state = -1;
			// 	// }
			// }

			// cout << " lihui obj:" << pObj->id << " state2:" << pObj->state << endl;
			break;
		}
		//parallel
		case 1:{
			// float time_now = ros::Time::now().toSec() * 1000;
			int predict_succ = Predict0(pObj,answer_predict,self_predict);
			// int predict_succ = Stop(pObj,answer_predict);
			if (To45(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 3;
				// }
			}
			else if (ToCross(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 2;
				// }
			}
			else if (ToStop(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				{
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = -1;
				}
			}
			break;
		}
		//cross
		case 2:{
			// float time_now = ros::Time::now().toSec() * 1000;
			int predict_succ = Predict90(pObj,answer_predict);
			if (To45(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 3;
				// }
			}
			else if (ToParallel(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time * 2)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 1;
				// }
			}
			else if (ToStop(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				{
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = -1;
				}
			}
			break;
		}
		//45 degree
		case 3:{
			// float time_now = ros::Time::now().toSec() * 1000;
			int predict_succ = Predict45(pObj,answer_predict);
			// int predict_succ = Predict45(pObj,answer_predict);
			if (ToStop(pObj))
			{
				pObj->stay_count++;
				if (pObj->stay_count > count_time)
				{
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = -1;
				}
			}
			else if (ToParallel(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time * 2)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 1;
				// }
			}
			else if (ToCross(pObj))
			{
				// pObj->stay_count++;
				// if (pObj->stay_count > count_time)
				// {
					pObj->stay_count = 0;
					// pObj->change_time = ros::Time::now().toSec() * 1000;
					pObj->state = 2;
				// }
			}
			break;
		}
		default:{
			int predict_succ = NoPredict(pObj,answer_predict);
			pObj->state = 0;
		}
	}
	return 1;
}

int direction_decide::GetDis(ivpredict::objbuffer *pObj, ivpredict::predictobj* self_predict){
	float frequency = 10.0;
	int refline_size = self_predict->positions.size();
	int buffer_size = pObj->positions.size();
	float mindis = 8888, final_dis;
	float temp_dis = 8888;
	int indexNearest = 0;
	//nearest point
	for (int i = 0; i < refline_size; ++i)
	{
		temp_dis = sqrt(pow(pObj->positions[buffer_size-1].xmean - self_predict->positions[i].xmean, 2) + pow(pObj->positions[buffer_size-1].ymean - self_predict->positions[i].ymean, 2));
		if (temp_dis < mindis)
        {
            mindis = temp_dis;
            indexNearest = i;
        }
	}
	pObj->rel_index = indexNearest;
	//nearest pose x y xg yg
	if (indexNearest > 0 && indexNearest < (refline_size-1))
	{
		ivpredict::predictpos pbj_pos, line_start_pos, line_end_pos, cross_pos;
		pbj_pos.xmean = pObj->positions[buffer_size-1].xmean;
		pbj_pos.ymean = pObj->positions[buffer_size-1].ymean;
		line_start_pos.xmean = self_predict->positions[indexNearest-1].xmean;
		line_start_pos.ymean = self_predict->positions[indexNearest-1].ymean;
		line_end_pos.xmean = self_predict->positions[indexNearest+1].xmean;
		line_end_pos.ymean = self_predict->positions[indexNearest+1].ymean;

		int cross_point_succ = GetCrossPoint(&pbj_pos, &line_start_pos, &line_end_pos, &cross_pos);
		pObj->positions[buffer_size-1].dis_x = cross_pos.xmean;
		pObj->positions[buffer_size-1].dis_y = cross_pos.ymean;
		final_dis = sqrt(pow((pbj_pos.xmean - cross_pos.xmean),2)+pow((pbj_pos.ymean - cross_pos.ymean),2));
		// cout << " lihui final dis:" << final_dis << endl;
		double angle = self_predict->heading - 90;
		float target_xg, target_yg;
		target_xg = cross_pos.xmean*cos(angle*3.1415/180.0) + cross_pos.xmean*sin(angle*3.1415/180.0);
    	target_yg = cross_pos.xmean*sin(angle*3.1415/180.0) - cross_pos.xmean*cos(angle*3.1415/180.0);
    	//translation transformation
    	pObj->positions[buffer_size-1].dis_xg = target_xg + self_predict->xvrel;
    	pObj->positions[buffer_size-1].dis_yg = target_yg + self_predict->yvrel;
		pObj->positions[buffer_size-1].dis_xg = self_predict->positions[indexNearest].xgmean;
		pObj->positions[buffer_size-1].dis_yg = self_predict->positions[indexNearest].ygmean;
	}
	else
	{
		pObj->positions[buffer_size-1].dis_x = self_predict->positions[indexNearest].xmean;
		pObj->positions[buffer_size-1].dis_y = self_predict->positions[indexNearest].ymean;
		pObj->positions[buffer_size-1].dis_xg = self_predict->positions[indexNearest].xgmean;
		pObj->positions[buffer_size-1].dis_yg = self_predict->positions[indexNearest].ygmean;
	}

	float temp_v = 0;
	if (buffer_size < 2)
	{
		pObj->positions[buffer_size-1].xdis = final_dis;
		pObj->positions[buffer_size-1].vdis = 0.0;
	}
	else
	{
		pObj->positions[buffer_size-1].xdis = (final_dis);// + pObj->positions[buffer_size-2].xdis + pObj->positions[buffer_size-2].vdis/frequency)/2;
		// cout << " lihui time stamp:" << (pObj->positions[buffer_size-1].timestamp - pObj->positions[0].timestamp).toSec() << endl;
		float temp_v = (pObj->positions[buffer_size-1].xdis - pObj->positions[buffer_size-3].xdis)/(pObj->positions[buffer_size-1].timestamp - pObj->positions[buffer_size-3].timestamp).toSec();
		//when cross line
		if (pObj->positions[buffer_size-2].vdis < 0 && temp_v > 0 && abs(pObj->positions[buffer_size-2].vdis > 0.3))
		{
			temp_v = (pObj->positions[buffer_size-1].xdis + pObj->positions[buffer_size-3].xdis)/(pObj->positions[buffer_size-1].timestamp - pObj->positions[buffer_size-3].timestamp).toSec();
		}
		pObj->positions[buffer_size-1].vdis = temp_v;
	}
}
int direction_decide::Predict90(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result){
	int buffer_size = pObj->positions.size();
	float frequency = 10.0;
	int total_output_point = 10*5;
	ivpredict::predictpos result_pos;

	predict_result->id = pObj->id;
	predict_result->length = pObj->length;
	predict_result->width = pObj->width;
	predict_result->height = pObj->height;
	predict_result->cell = pObj->cell;
	predict_result->time_interval = 1./10.0 ;
	predict_result->steps = total_output_point;
	predict_result->xvrel = pObj->xvrel;
	predict_result->yvrel = pObj->yvrel;
	predict_result->vrel = pObj->vrel;
	predict_result->type = pObj->property;
	predict_result->v_abs = pObj->vabs;
	predict_result->state = pObj->state;

	// cout << " lihui obj pos now = " << obj_pos.xmean << " " << obj_pos.ymean << endl;
	// cout << " lihui line pos now = " << line_pos.xmean << " " << line_pos.ymean << endl;
	// float speed_dis =pObj->vabs;
	float speed_dis = abs(pObj->positions[buffer_size-1].vdis)/frequency;
	for (int i = 0; i < total_output_point; ++i)
	{
		result_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean + speed_dis*cos(pObj->theta_toline)*i;
		result_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean + speed_dis*sin(pObj->theta_toline)*i;
		predict_result->positions.push_back(result_pos);
		// cout << " lihui pos:" << i << " xmean:" << result_pos.xmean << " ymean:" << result_pos.ymean << endl;
	}

	return 1;
}

int direction_decide::Stop(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result){
	int buffer_size = pObj->positions.size();
	// float theta = atan2((pObj->positions[buffer_size-1].ymean-pObj->positions[buffer_size-1].dis_y),(pObj->positions[buffer_size-1].xmean-pObj->positions[buffer_size-1].dis_x));
	int total_output_point = 10*5;
	ivpredict::predictpos result_pos;

	predict_result->id = pObj->id;

	predict_result->length = pObj->length;
	predict_result->width = pObj->width;
	predict_result->height = pObj->height;
	predict_result->cell = pObj->cell;
	predict_result->time_interval = 1./10.0 ;
	predict_result->steps = total_output_point;
	predict_result->xvrel = pObj->xvrel;
	predict_result->yvrel = pObj->yvrel;
	predict_result->vrel = pObj->vrel;
	predict_result->type = pObj->property;
	predict_result->v_abs = 0.0;
	predict_result->state = pObj->state;

	for (int i = 0; i < total_output_point; ++i)
	{
		result_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean;
		result_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean;
		predict_result->positions.push_back(result_pos);
	}
	return 1;
}

int direction_decide::NoPredict(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result){
	int buffer_size = pObj->positions.size();
	float theta = atan2((pObj->positions[buffer_size-1].ymean-pObj->positions[buffer_size-1].dis_y),(pObj->positions[buffer_size-1].xmean-pObj->positions[buffer_size-1].dis_x));
	int total_output_point = 10*5;
	ivpredict::predictpos result_pos;

	predict_result->id = pObj->id;

	predict_result->length = pObj->length;
	predict_result->width = pObj->width;
	predict_result->height = pObj->height;
	predict_result->cell = pObj->cell;
	predict_result->time_interval = 1./10.0 ;
	predict_result->steps = total_output_point;
	predict_result->xvrel = pObj->xvrel;
	predict_result->yvrel = pObj->yvrel;
	predict_result->vrel = pObj->vrel;
	predict_result->type = pObj->property;
	predict_result->v_abs = pObj->vabs;
	predict_result->state = pObj->state;
	// pObj->direction_y = true;
	// pObj->direction_x = true;

	for (int i = 0; i < total_output_point; ++i)
	{
		result_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean;
		result_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean;
		predict_result->positions.push_back(result_pos);
	}
	return 1;
}

int direction_decide::Predict0(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result, ivpredict::predictobj* self_predict){
	float frequency = 10.0;
	int buffer_size = pObj->positions.size();
	int total_output_point = 10*5;
	ivpredict::predictpos result_pos;

	predict_result->id = pObj->id;
	predict_result->length = pObj->length;
	predict_result->width = pObj->width;
	predict_result->height = pObj->height;
	predict_result->cell = pObj->cell;
	predict_result->xvrel = pObj->xvrel;
	predict_result->yvrel = pObj->yvrel;
	predict_result->vrel = pObj->vrel;
	predict_result->type = pObj->property;
	predict_result->v_abs = pObj->vabs;
	predict_result->state = pObj->state;

	float dx = pObj->positions[pObj->positions.size()-1].xmean - self_predict->positions[pObj->rel_index].xmean;
	float dy = pObj->positions[pObj->positions.size()-1].ymean - self_predict->positions[pObj->rel_index].ymean;
	// float direction = sqrt(pow( (self_predict->positions[index + 3].xmean - pObj->positions[pObj->positions.size()-1].xmean),2)+pow( (self_predict->positions[index + 3].ymean - pObj->positions[pObj->positions.size()-1].ymean ),2));
	// float direction2 = sqrt(pow( (self_predict->positions[index + 3].xmean - pObj->positions[pObj->positions.size()-5].xmean),2)+pow( (self_predict->positions[index + 3].ymean - pObj->positions[pObj->positions.size()-5].ymean ),2));
	// float speed_delta = pObj->vabs/10.0;
	float speed_delta;
	// for (int i = 0; i < (buffer_size-1); ++i)
	// {
	// 	temp_speed += sqrt(pow((pObj->positions[i].dis_xg - pObj->positions[i+1].dis_xg),2)+pow((pObj->positions[i].dis_yg - pObj->positions[i+1].dis_yg),2));
	// }
	speed_delta = pObj->positions[buffer_size-1].vydis/float(frequency);

	result_pos.xmean = pObj->positions[buffer_size-1].xmean;
	result_pos.ymean = pObj->positions[buffer_size-1].ymean;
	predict_result->positions.push_back(result_pos);
	if (pObj->direction_y)
	{
		for (int i = pObj->rel_index; i < self_predict->positions.size(); ++i)
		{
			float pointx = self_predict->positions[i].xmean;
			float pointy = self_predict->positions[i].ymean;
			float dis = sqrt(pow( (pointx - pObj->positions[pObj->positions.size()-1].dis_x),2)+pow( (pointy - pObj->positions[pObj->positions.size()-1].dis_y ),2));
			if (dis < speed_delta){
		
				continue;
			}
			if (dis > speed_delta*50 && predict_result->positions.size() > 1)
			{
				break;
			}
			result_pos.xmean = pointx + dx;
			result_pos.ymean = pointy + dy;
			predict_result->positions.push_back(result_pos);
		}
	}
	else
	{
		for (int i = pObj->rel_index; i >= 0; --i)
		{
			float pointx = self_predict->positions[i].xmean;
			float pointy = self_predict->positions[i].ymean;
			float dis = sqrt(pow( (pointx - pObj->positions[pObj->positions.size()-1].dis_x),2)+pow( (pointy - pObj->positions[pObj->positions.size()-1].dis_y ),2));
			if (dis < speed_delta)
			{
				continue;
			}
			if (dis > speed_delta*50 && predict_result->positions.size() > 1)
			{
				break;
			}
			result_pos.xmean = pointx + dx;
			result_pos.ymean = pointy + dy;
			predict_result->positions.push_back(result_pos);
		}
	}

	// cout << " lihui obj:" << pObj->id << " size:" << predict_result->positions.size() << endl;
	predict_result->steps = predict_result->positions.size();
	predict_result->time_interval = 5.0/float(total_output_point) ;

	return 1;
}

int direction_decide::Predict45(ivpredict::objbuffer *pObj, ivpredict::predictobj* predict_result){
	int buffer_size = pObj->positions.size();
	float frequency = 10.0;
	int total_output_point = 10*5;
	ivpredict::predictpos result_pos;

	predict_result->id = pObj->id;
	predict_result->length = pObj->length;
	predict_result->width = pObj->width;
	predict_result->height = pObj->height;
	predict_result->cell = pObj->cell;
	predict_result->time_interval = 1./10.0 ;
	predict_result->steps = total_output_point;
	predict_result->xvrel = pObj->xvrel;
	predict_result->yvrel = pObj->yvrel;
	predict_result->vrel = pObj->vrel;
	predict_result->type = pObj->property;
	predict_result->v_abs = pObj->vabs;
	predict_result->state = pObj->state;

	// cout << " lihui obj pos now = " << obj_pos.xmean << " " << obj_pos.ymean << endl;
	
	// float speed_dis =pObj->vabs;
	int temp_dir_x = -1,temp_dir_y = -1;
	// if (pObj->direction_x)
	{
		temp_dir_y = 1;
		if (pObj->direction_y)
		{
		temp_dir_y = -1;
		}
	}
	


	float theta_45 = pObj->theta_toline + 45.0/180.0*3.14*temp_dir_x*temp_dir_y;
	if (theta_45 > 1.5707 && pObj->direction_y)
	{
		theta_45 -= 1.5707;
	}
	//cout << " lihui angle =" << pObj->theta_toline << " " << theta_45 << " dirx:" << temp_dir_x << " drry:" << temp_dir_y << endl;
	float speed_dis = abs(pObj->positions[buffer_size-1].vdis)/frequency;
	for (int i = 0; i < total_output_point; ++i)
	{
		result_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean + speed_dis*cos(theta_45)*i;
		result_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean + speed_dis*sin(theta_45)*i;
		predict_result->positions.push_back(result_pos);
		// cout << " lihui pos:" << i << " xmean:" << result_pos.xmean << " ymean:" << result_pos.ymean << endl;
	}

	return 1;
}

bool direction_decide::ToStop(ivpredict::objbuffer *pObj)
{
	int temp_buffer_size = pObj->positions.size();
	if (
		abs(pObj->positions[temp_buffer_size-1].vdis) < 0.03 &&
		abs(pObj->positions[temp_buffer_size-2].vdis) < 0.03 &&
		abs(pObj->positions[temp_buffer_size-3].vdis) < 0.03 &&
		abs(pObj->positions[temp_buffer_size-1].vydis)< 0.5 &&
		abs(pObj->positions[temp_buffer_size-2].vydis)< 0.5 &&
		abs(pObj->positions[temp_buffer_size-3].vydis)< 0.5
		 )
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool direction_decide::ToCross(ivpredict::objbuffer *pObj)
{
	int temp_buffer_size = pObj->positions.size();
	float vy = abs(pObj->positions[temp_buffer_size-1].vydis);
	float vx = abs(pObj->positions[temp_buffer_size-1].vdis);
	if(
		(
			(//three speed over	
				abs(pObj->positions[temp_buffer_size-1].vdis) > 0.13 &&
				abs(pObj->positions[temp_buffer_size-2].vdis) > 0.12 && 
				abs(pObj->positions[temp_buffer_size-3].vdis) > 0.11
			) ||
			//abs cross
			abs(pObj->positions[temp_buffer_size-1].vdis) > 0.2
			||
			(
				abs(pObj->positions[temp_buffer_size-1].vdis) > 0.15 &&
				abs(pObj->positions[temp_buffer_size-2].vdis) > 0.12
			)
		)
		&& abs(vx/vy) > 10

		// abs(pObj->positions[temp_buffer_size-1].vdis)/abs(pObj->positions[temp_buffer_size-1].vabs) > 1
	)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool direction_decide::ToParallel(ivpredict::objbuffer *pObj)
{
	
	int temp_buffer_size = pObj->positions.size();
	float vy = abs(pObj->positions[temp_buffer_size-1].vydis);
	if (
			(
				abs(pObj->positions[temp_buffer_size-1].vdis) < 0.05 && 
				abs(pObj->positions[temp_buffer_size-2].vdis) < 0.05 &&
				abs(pObj->positions[temp_buffer_size-3].vdis) < 0.05 &&
				(vy > 1) 
			//  && abs(vx/vy) < 0.1
			)
			||
			vy > 1.5
		)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool direction_decide::To45(ivpredict::objbuffer *pObj)
{
	// pObj->positions[buffer_size-1].vydis abs(pObj->positions[buffer_size-1].vdis)
	int temp_buffer_size = pObj->positions.size();
	float vy = abs(pObj->positions[temp_buffer_size-1].vydis);
	float vx = abs(pObj->positions[temp_buffer_size-1].vdis);
	if (
		pObj->heading < 70 &&
		pObj->heading > 30 &&
		vx > 0.3 &&
		vy > 0.3
		 )
	{
		return true;
	}
	else
	{
		return false;
	}
}

int direction_decide::GetCrossPoint(ivpredict::predictpos* obj_pos, ivpredict::predictpos* start_pos, ivpredict::predictpos* end_pos, ivpredict::predictpos* cross_pos){
	float k1,b1,k2,b2;
	k1 = (end_pos->ymean - start_pos->ymean)/(end_pos->xmean - start_pos->xmean);
	b1 = start_pos->ymean - k1*start_pos->xmean;
	k2 = -1.0/k1;
	b2 = obj_pos->ymean - k2*obj_pos->xmean;
	cross_pos->xmean = (b2 - b1)/(k1 - k2);
	cross_pos->ymean = k1*cross_pos->xmean + b1;
	return 1;
}

int direction_decide::UpdateStates(ivpredict::objbuffer *pObj, ivpredict::predictobj* self_predict)
{
	int buffer_size = pObj->positions.size();
	ivpredict::predictpos obj_pos, line_pos;

	//x update theta
	float theta;
	bool direction_last = pObj->direction_x;
	bool direction_now;
	float speed_dis = 0,temp_dis = 0; 
	// cout << " lihui obj:" << pObj->id << " speed:" << pObj->positions[buffer_size-1].vdis << endl;
	if (pObj->positions[buffer_size-1].vdis < 0)
	{
		direction_now = true;
	}
	else
	{
		direction_now = false;
	}

	if (direction_last && !direction_now || (!direction_last && direction_now))
	{
		pObj->direction_x_change++;
		if (pObj->direction_x_change > 1)
		{
			pObj->direction_x_change = 0;
			pObj->direction_x = direction_now;
		}
	}
	pObj->direction_x = direction_now;

	obj_pos.xmean = pObj->positions[pObj->positions.size()-1].xmean;
	obj_pos.ymean = pObj->positions[pObj->positions.size()-1].ymean;
	line_pos.xmean = pObj->positions[pObj->positions.size()-1].dis_x;
	line_pos.ymean = pObj->positions[pObj->positions.size()-1].dis_y;
	
	// cout << " lihui obj:" << pObj->id << " theta:" << pObj->theta_toline << endl;
	if (buffer_size == 1)
	{
		pObj->theta_toline = theta = atan2((line_pos.ymean-obj_pos.ymean),(line_pos.xmean-obj_pos.xmean));
	}
	if (pObj->direction_x)
	{
		theta = atan2((line_pos.ymean-obj_pos.ymean),(line_pos.xmean-obj_pos.xmean));
	
		if (abs(theta - pObj->theta_toline) > 1 && pObj->positions[buffer_size-1].xdis < 0.8)
		{
			theta = atan2((obj_pos.ymean - line_pos.ymean),(obj_pos.xmean - line_pos.xmean));
		}
	}
	else
	{
		// theta = pObj->theta_toline;
		theta = atan2((obj_pos.ymean - line_pos.ymean),(obj_pos.xmean - line_pos.xmean));
	}

	// if (pObj->positions[buffer_size-1].xdis < 0.3)
	pObj->theta_toline = theta;

	//y_update abs_speed_y
	int last_index_abs;
	float speed_delta,temp_speed = 0;
	if (pObj->abs_index == 0)
	{
		last_index_abs = self_predict->state + pObj->rel_index;
		speed_delta = 0;
	}
	else
	{
		last_index_abs = pObj->abs_index;
	}

	int status_now = self_predict->state + pObj->rel_index;

	//speed way 1
	if (buffer_size < 2)
	{
		temp_speed = 0;
	}
	else
	{
		temp_speed = sqrt(pow((pObj->positions[buffer_size-1].dis_xg - pObj->positions[0].dis_xg),2)+pow((pObj->positions[buffer_size-1].dis_yg - pObj->positions[0].dis_yg),2));
		speed_delta = temp_speed/(pObj->positions[buffer_size-1].timestamp - pObj->positions[buffer_size-2].timestamp).toSec()/float(buffer_size-1);
		pObj->positions[buffer_size-1].vydis = speed_delta;
	}

	//speed way 2
	// if (buffer_size < 2)
	// {
	// 	temp_speed = 0;
	// }
	// else
	// {
	// 	temp_speed = sqrt(pow((self_predict->positions[pObj->rel_index].xgmean - self_predict->positions[last_index_abs - self_predict->state].xgmean),2)+pow((self_predict->positions[pObj->rel_index].ygmean - self_predict->positions[last_index_abs - self_predict->state].ygmean),2));
	// 	speed_delta = temp_speed/(pObj->positions[buffer_size-1].timestamp - pObj->positions[buffer_size-2].timestamp).toSec();
	// 	pObj->positions[buffer_size-1].vydis = speed_delta;
	// }

	pObj->abs_index = status_now;
	bool direction_y = pObj->direction_y;
	bool direction_y_now;
	//get direction now
	if (status_now > last_index_abs)
	{
		direction_y_now = true;
	}
	else if (status_now < last_index_abs)
	{
		direction_y_now = false;
	}
	else
	{
		direction_y_now = direction_y;
	}
	//change 3 times
	// cout << pObj->id << " lihui direction: " << direction_y << " now:" << direction_y_now << endl;
	if (direction_y && !direction_y_now || (!direction_y && direction_y_now))
	{
		pObj->direction_y_change++;
		if (pObj->direction_y_change > 1)
		{
			direction_y = direction_y_now;
			pObj->direction_y_change = 0;
		}
	}
	pObj->direction_y = direction_y;

	//heading
	float vy = abs(pObj->positions[buffer_size-1].vydis);
	float vx = abs(pObj->positions[buffer_size-1].vdis);
	float heading_now = atan2(vx,vy)*180.0/3.14, temp_heading = 0;
	if (buffer_size < 3)
	{
		pObj->positions[buffer_size-1].heading = 0.0;
	}
	else
	{
		for (int i = 0; i < buffer_size; ++i)
		{
			temp_heading += pObj->positions[i].heading;
		}
	}
	
	pObj->heading = (temp_heading/float(buffer_size) + heading_now)/2.0;
	// cout << " lihui obj id:" << pObj->id << " v-dis:" << pObj->positions[buffer_size-1].vdis << " dis:" << pObj->positions[buffer_size-1].xdis 
	// << " vx:" << vx << " vy:" << vy << " state:" << pObj->state  << " angle" << pObj->heading
	// << endl;
	return 1;
}
