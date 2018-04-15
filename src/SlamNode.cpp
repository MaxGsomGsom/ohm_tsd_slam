/*
 * SlamNode.cpp
 *
 *  Created on: 05.05.2014
 *      Author: phil
 */

#include "SlamNode.h"
#include "ThreadMapping.h"
#include "ThreadGrid.h"
#include <string>

#include "obcore/math/mathbase.h"

using namespace std;
using namespace ros::names;


namespace ohm_tsd_slam
{
SlamNode::SlamNode(void)
{
  ros::NodeHandle prvNh("~");
  int robotNbr;
  double gridPublishInterval;
  double loopRateVar;
  double truncationRadius;
  double cellSize;
  int octaveFactor;
  double xOffset;
  double yOffset;
  string topicLaser;
  string topicServiceStartStop;
  string curRobotName;

  prvNh.param<int>("robot_nbr", robotNbr, 1);
  prvNh.param<int>("map_size", octaveFactor, 10);
  prvNh.param<double>("x_offset", xOffset, 0);
  prvNh.param<double>("y_offset", yOffset, 0);
  prvNh.param<double>("cellsize", cellSize, 0.025);
  prvNh.param<double>("truncation_radius", truncationRadius, 3);
  prvNh.param<double>("occ_grid_time_interval", gridPublishInterval, 2.0);
  prvNh.param<double>("loop_rate", loopRateVar, 30.0);
  prvNh.param<string>("laser_topic", topicLaser, "scan");
  prvNh.param<string>("start_service_name", topicServiceStartStop, "start_slam");
  prvNh.param<string>("cur_robot_name", curRobotName, "");

  _loopRate = new ros::Rate(loopRateVar);
  _gridInterval = new ros::Duration(gridPublishInterval);

  if(octaveFactor > 15)
  {
    ROS_ERROR_STREAM("Error! Unknown map size -> set to default!" << endl);
    octaveFactor = 10;
  }
  //instanciate representation
  _grid = new obvious::TsdGrid(cellSize, obvious::LAYOUT_32x32, static_cast<obvious::EnumTsdGridLayout>(octaveFactor));  //obvious::LAYOUT_8192x8192
  _grid->setMaxTruncation(truncationRadius * cellSize);
  unsigned int cellsPerSide = pow(2, octaveFactor);
  double sideLength = static_cast<double>(cellsPerSide) * cellSize;
  ROS_INFO_STREAM("Creating representation with " << cellsPerSide << "x" << cellsPerSide << "cells, representating " <<
                  sideLength << "x" << sideLength << "m^2" << endl);
  //instanciate mapping threads
  _threadMapping = new ThreadMapping(_grid);
  _threadGrid    = new ThreadGrid(_grid, &_nh, xOffset, yOffset);

  ThreadLocalize* threadLocalize = NULL;
  TaggedSubscriber subs;
  string nameSpace = "";

  //instanciate localization threads
  if(robotNbr == 1)  //single slam
  {
    threadLocalize = new ThreadLocalize(_grid, _threadMapping, &_nh, nameSpace, xOffset, yOffset);
    subs = TaggedSubscriber("/"+nameSpace+"/"+topicLaser, *threadLocalize, _nh);
    subs.switchOn();
    _subsLaser.push_back(subs);
    _localizers.push_back(threadLocalize);
    ROS_INFO_STREAM("Single SLAM started" << endl);
  }
  else
  {
    for(int i = 0; i < robotNbr; i++)   //multi slam
    {
      prvNh.param<string>("robot_name_" + to_string(i), nameSpace, "robot" + to_string(i));
      threadLocalize = new ThreadLocalize(_grid, _threadMapping, &_nh, nameSpace, xOffset, yOffset);
      subs = TaggedSubscriber("/"+nameSpace+"/"+topicLaser, *threadLocalize, _nh);
      subs.switchOn();
      _subsLaser.push_back(subs);
      _localizers.push_back(threadLocalize);
      ROS_INFO_STREAM("Started thread for " << nameSpace << endl);
    }
    ROS_INFO_STREAM("Multi SLAM started");
  }
  _serviceStartStopSLAM = _nh.advertiseService("/"+curRobotName+"/"+topicServiceStartStop, &SlamNode::callBackServiceStartStopSLAM, this);
}

SlamNode::~SlamNode()
{
  //stop all localization threads
  for(vector<ThreadLocalize*>::iterator iter = _localizers.begin(); iter < _localizers.end(); iter++)
  {
    (*iter)->terminateThread();
    while((*iter)->alive(THREAD_TERM_MS))
      usleep(THREAD_TERM_MS);
    delete *iter;
  }
  delete _loopRate;
  delete _gridInterval;
  //stop mapping threads
  _threadGrid->terminateThread();
  while(_threadGrid->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadGrid;
  _threadMapping->terminateThread();
  while(_threadMapping->alive(THREAD_TERM_MS))
    usleep(THREAD_TERM_MS);
  delete _threadMapping;
  delete _grid;
}

void SlamNode::timedGridPub(void)
{
  static ros::Time lastMap = ros::Time::now();
  ros::Time curTime = ros::Time::now();
  if((curTime - lastMap).toSec() > _gridInterval->toSec())
  {
    _threadGrid->unblock();
    lastMap = ros::Time::now();
  }
}

void SlamNode::run(void)
{
  while(ros::ok())
  {
    ros::spinOnce();
    this->timedGridPub();
    _loopRate->sleep();
  }
}

bool SlamNode::callBackServiceStartStopSLAM(ohm_tsd_slam::StartStopSLAM::Request& req, ohm_tsd_slam::StartStopSLAM::Response& res)
{
  TaggedSubscriber* subsCur = NULL;
  for(auto iter = _subsLaser.begin(); iter < _subsLaser.end(); iter++)
  {
    if(iter->equal(req.topic))
      subsCur = &*iter;
  }
  if(!subsCur)
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Error! Topic " << req.topic << " invalid!");
    return false;
  }
  if(req.startStop == req.START)
  {
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " Started SLAM for topic " << req.topic);
    subsCur->switchOn();
  }
  else if(req.startStop == req.STOP)
  {
    ROS_INFO_STREAM(__PRETTY_FUNCTION__ << " Stopped SLAM for topic " << req.topic);
    subsCur->switchOff();
  }
  else
  {
    ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Error. Unknown request for service");
    return false;
  }
  return true;
}

} /* namespace ohm_tsd_slam */
