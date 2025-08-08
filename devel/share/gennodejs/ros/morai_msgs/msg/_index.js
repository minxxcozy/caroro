
"use strict";

let SyncModeInfo = require('./SyncModeInfo.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let SensorPosControl = require('./SensorPosControl.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let EventInfo = require('./EventInfo.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let RadarDetection = require('./RadarDetection.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let VelocityCmd = require('./VelocityCmd.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let CollisionData = require('./CollisionData.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let GPSMessage = require('./GPSMessage.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let ExternalForce = require('./ExternalForce.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let WaitForTick = require('./WaitForTick.js');
let CtrlCmd = require('./CtrlCmd.js');
let VehicleSpec = require('./VehicleSpec.js');
let GVStateCmd = require('./GVStateCmd.js');
let TOF = require('./TOF.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let CMDConveyor = require('./CMDConveyor.js');
let SaveSensorData = require('./SaveSensorData.js');
let WheelControl = require('./WheelControl.js');
let RobotOutput = require('./RobotOutput.js');
let IntscnTL = require('./IntscnTL.js');
let PRStatus = require('./PRStatus.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let ObjectStatus = require('./ObjectStatus.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let TrafficLight = require('./TrafficLight.js');
let DdCtrlCmd = require('./DdCtrlCmd.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let Obstacle = require('./Obstacle.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let DillyCmd = require('./DillyCmd.js');
let ManipulatorControl = require('./ManipulatorControl.js');
let ShipState = require('./ShipState.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let Conveyor = require('./Conveyor.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let PREvent = require('./PREvent.js');
let IntersectionControl = require('./IntersectionControl.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let VehicleCollision = require('./VehicleCollision.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let ERP42Info = require('./ERP42Info.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let ReplayInfo = require('./ReplayInfo.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let RadarDetections = require('./RadarDetections.js');
let MapSpec = require('./MapSpec.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let Transforms = require('./Transforms.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let GhostMessage = require('./GhostMessage.js');
let Lamps = require('./Lamps.js');
let Obstacles = require('./Obstacles.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let SVADC = require('./SVADC.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let RobotState = require('./RobotState.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');

module.exports = {
  SyncModeInfo: SyncModeInfo,
  FaultInjection_Tire: FaultInjection_Tire,
  ObjectStatusExtended: ObjectStatusExtended,
  SensorPosControl: SensorPosControl,
  ObjectStatusList: ObjectStatusList,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  VehicleCollisionData: VehicleCollisionData,
  SyncModeCmdResponse: SyncModeCmdResponse,
  EventInfo: EventInfo,
  MoraiTLIndex: MoraiTLIndex,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  SkateboardStatus: SkateboardStatus,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  SyncModeSetGear: SyncModeSetGear,
  VehicleSpecIndex: VehicleSpecIndex,
  SyncModeResultResponse: SyncModeResultResponse,
  RadarDetection: RadarDetection,
  WaitForTickResponse: WaitForTickResponse,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  VelocityCmd: VelocityCmd,
  FaultInjection_Sensor: FaultInjection_Sensor,
  EgoVehicleStatus: EgoVehicleStatus,
  SyncModeAddObject: SyncModeAddObject,
  CollisionData: CollisionData,
  ObjectStatusListExtended: ObjectStatusListExtended,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  NpcGhostCmd: NpcGhostCmd,
  PRCtrlCmd: PRCtrlCmd,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  MultiPlayEventRequest: MultiPlayEventRequest,
  GPSMessage: GPSMessage,
  SyncModeCmd: SyncModeCmd,
  ScenarioLoad: ScenarioLoad,
  GetTrafficLightStatus: GetTrafficLightStatus,
  ExternalForce: ExternalForce,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  WaitForTick: WaitForTick,
  CtrlCmd: CtrlCmd,
  VehicleSpec: VehicleSpec,
  GVStateCmd: GVStateCmd,
  TOF: TOF,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  IntersectionStatus: IntersectionStatus,
  CMDConveyor: CMDConveyor,
  SaveSensorData: SaveSensorData,
  WheelControl: WheelControl,
  RobotOutput: RobotOutput,
  IntscnTL: IntscnTL,
  PRStatus: PRStatus,
  WoowaDillyStatus: WoowaDillyStatus,
  ObjectStatus: ObjectStatus,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  TrafficLight: TrafficLight,
  DdCtrlCmd: DdCtrlCmd,
  FaultInjection_Response: FaultInjection_Response,
  Obstacle: Obstacle,
  MapSpecIndex: MapSpecIndex,
  DillyCmd: DillyCmd,
  ManipulatorControl: ManipulatorControl,
  ShipState: ShipState,
  SyncModeRemoveObject: SyncModeRemoveObject,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  Conveyor: Conveyor,
  MultiEgoSetting: MultiEgoSetting,
  PREvent: PREvent,
  IntersectionControl: IntersectionControl,
  FaultStatusInfo: FaultStatusInfo,
  VehicleCollision: VehicleCollision,
  SetTrafficLight: SetTrafficLight,
  FaultInjection_Controller: FaultInjection_Controller,
  MoraiSrvResponse: MoraiSrvResponse,
  ERP42Info: ERP42Info,
  GVDirectCmd: GVDirectCmd,
  ReplayInfo: ReplayInfo,
  ShipCtrlCmd: ShipCtrlCmd,
  NpcGhostInfo: NpcGhostInfo,
  RadarDetections: RadarDetections,
  MapSpec: MapSpec,
  GeoVector3Message: GeoVector3Message,
  MoraiTLInfo: MoraiTLInfo,
  Transforms: Transforms,
  DillyCmdResponse: DillyCmdResponse,
  GhostMessage: GhostMessage,
  Lamps: Lamps,
  Obstacles: Obstacles,
  MoraiSimProcStatus: MoraiSimProcStatus,
  SVADC: SVADC,
  MoraiSimProcHandle: MoraiSimProcHandle,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  RobotState: RobotState,
  MultiPlayEventResponse: MultiPlayEventResponse,
};
