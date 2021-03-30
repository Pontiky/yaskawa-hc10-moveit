
"use strict";

let GetRobotInfo = require('./GetRobotInfo.js')
let StopMotion = require('./StopMotion.js')
let StartMotion = require('./StartMotion.js')
let SetRemoteLoggerLevel = require('./SetRemoteLoggerLevel.js')
let CmdJointTrajectory = require('./CmdJointTrajectory.js')
let SetDrivePower = require('./SetDrivePower.js')

module.exports = {
  GetRobotInfo: GetRobotInfo,
  StopMotion: StopMotion,
  StartMotion: StartMotion,
  SetRemoteLoggerLevel: SetRemoteLoggerLevel,
  CmdJointTrajectory: CmdJointTrajectory,
  SetDrivePower: SetDrivePower,
};
