
"use strict";

let DdCtrlCmd = require('./DdCtrlCmd.js');
let PRCtrlCmd = require('./PRCtrlCmd.js');
let DillyCmd = require('./DillyCmd.js');
let GeoVector3Message = require('./GeoVector3Message.js');
let RadarDetection = require('./RadarDetection.js');
let IntersectionControl = require('./IntersectionControl.js');
let Conveyor = require('./Conveyor.js');
let VelocityCmd = require('./VelocityCmd.js');
let TOF = require('./TOF.js');
let DillyCmdResponse = require('./DillyCmdResponse.js');
let Lamps = require('./Lamps.js');
let SaveSensorData = require('./SaveSensorData.js');
let VehicleCollision = require('./VehicleCollision.js');
let SyncModeInfo = require('./SyncModeInfo.js');
let WoowaDillyStatus = require('./WoowaDillyStatus.js');
let UGVServeSkidCtrlCmd = require('./UGVServeSkidCtrlCmd.js');
let SkateboardStatus = require('./SkateboardStatus.js');
let ObjectStatus = require('./ObjectStatus.js');
let SyncModeAddObject = require('./SyncModeAddObject.js');
let SensorPosControl = require('./SensorPosControl.js');
let VehicleSpecIndex = require('./VehicleSpecIndex.js');
let SyncModeRemoveObject = require('./SyncModeRemoveObject.js');
let SyncModeScenarioLoad = require('./SyncModeScenarioLoad.js');
let FaultInjection_Tire = require('./FaultInjection_Tire.js');
let CMDConveyor = require('./CMDConveyor.js');
let FaultInjection_Controller = require('./FaultInjection_Controller.js');
let MoraiTLInfo = require('./MoraiTLInfo.js');
let FaultInjection_Sensor = require('./FaultInjection_Sensor.js');
let IntscnTL = require('./IntscnTL.js');
let FaultStatusInfo_Vehicle = require('./FaultStatusInfo_Vehicle.js');
let SyncModeSetGear = require('./SyncModeSetGear.js');
let MoraiTLIndex = require('./MoraiTLIndex.js');
let ReplayInfo = require('./ReplayInfo.js');
let Obstacles = require('./Obstacles.js');
let MoraiSrvResponse = require('./MoraiSrvResponse.js');
let FaultStatusInfo = require('./FaultStatusInfo.js');
let EgoVehicleStatusExtended = require('./EgoVehicleStatusExtended.js');
let EventInfo = require('./EventInfo.js');
let NpcGhostCmd = require('./NpcGhostCmd.js');
let FaultStatusInfo_Overall = require('./FaultStatusInfo_Overall.js');
let FaultStatusInfo_Sensor = require('./FaultStatusInfo_Sensor.js');
let GhostMessage = require('./GhostMessage.js');
let SetTrafficLight = require('./SetTrafficLight.js');
let WaitForTickResponse = require('./WaitForTickResponse.js');
let GPSMessage = require('./GPSMessage.js');
let PRStatus = require('./PRStatus.js');
let MoraiSimProcHandle = require('./MoraiSimProcHandle.js');
let SkidSteer6wUGVStatus = require('./SkidSteer6wUGVStatus.js');
let ObjectStatusListExtended = require('./ObjectStatusListExtended.js');
let EgoDdVehicleStatus = require('./EgoDdVehicleStatus.js');
let CollisionData = require('./CollisionData.js');
let MapSpecIndex = require('./MapSpecIndex.js');
let MultiPlayEventRequest = require('./MultiPlayEventRequest.js');
let IntersectionStatus = require('./IntersectionStatus.js');
let ScenarioLoad = require('./ScenarioLoad.js');
let RobotState = require('./RobotState.js');
let FaultInjection_Response = require('./FaultInjection_Response.js');
let SyncModeCmd = require('./SyncModeCmd.js');
let SyncModeResultResponse = require('./SyncModeResultResponse.js');
let ShipState = require('./ShipState.js');
let WaitForTick = require('./WaitForTick.js');
let PREvent = require('./PREvent.js');
let CtrlCmd = require('./CtrlCmd.js');
let SkidSteer6wUGVCtrlCmd = require('./SkidSteer6wUGVCtrlCmd.js');
let MapSpec = require('./MapSpec.js');
let RobotOutput = require('./RobotOutput.js');
let SyncModeCmdResponse = require('./SyncModeCmdResponse.js');
let TrafficLight = require('./TrafficLight.js');
let MoraiSimProcStatus = require('./MoraiSimProcStatus.js');
let VehicleCollisionData = require('./VehicleCollisionData.js');
let GVDirectCmd = require('./GVDirectCmd.js');
let NpcGhostInfo = require('./NpcGhostInfo.js');
let RadarDetections = require('./RadarDetections.js');
let Transforms = require('./Transforms.js');
let WheelControl = require('./WheelControl.js');
let ObjectStatusExtended = require('./ObjectStatusExtended.js');
let GVStateCmd = require('./GVStateCmd.js');
let SyncModeCtrlCmd = require('./SyncModeCtrlCmd.js');
let MultiEgoSetting = require('./MultiEgoSetting.js');
let EgoVehicleStatus = require('./EgoVehicleStatus.js');
let ObjectStatusList = require('./ObjectStatusList.js');
let ManipulatorControl = require('./ManipulatorControl.js');
let ExternalForce = require('./ExternalForce.js');
let ERP42Info = require('./ERP42Info.js');
let VehicleSpec = require('./VehicleSpec.js');
let GetTrafficLightStatus = require('./GetTrafficLightStatus.js');
let Obstacle = require('./Obstacle.js');
let SVADC = require('./SVADC.js');
let SkateboardCtrlCmd = require('./SkateboardCtrlCmd.js');
let MultiPlayEventResponse = require('./MultiPlayEventResponse.js');
let ShipCtrlCmd = require('./ShipCtrlCmd.js');

module.exports = {
  DdCtrlCmd: DdCtrlCmd,
  PRCtrlCmd: PRCtrlCmd,
  DillyCmd: DillyCmd,
  GeoVector3Message: GeoVector3Message,
  RadarDetection: RadarDetection,
  IntersectionControl: IntersectionControl,
  Conveyor: Conveyor,
  VelocityCmd: VelocityCmd,
  TOF: TOF,
  DillyCmdResponse: DillyCmdResponse,
  Lamps: Lamps,
  SaveSensorData: SaveSensorData,
  VehicleCollision: VehicleCollision,
  SyncModeInfo: SyncModeInfo,
  WoowaDillyStatus: WoowaDillyStatus,
  UGVServeSkidCtrlCmd: UGVServeSkidCtrlCmd,
  SkateboardStatus: SkateboardStatus,
  ObjectStatus: ObjectStatus,
  SyncModeAddObject: SyncModeAddObject,
  SensorPosControl: SensorPosControl,
  VehicleSpecIndex: VehicleSpecIndex,
  SyncModeRemoveObject: SyncModeRemoveObject,
  SyncModeScenarioLoad: SyncModeScenarioLoad,
  FaultInjection_Tire: FaultInjection_Tire,
  CMDConveyor: CMDConveyor,
  FaultInjection_Controller: FaultInjection_Controller,
  MoraiTLInfo: MoraiTLInfo,
  FaultInjection_Sensor: FaultInjection_Sensor,
  IntscnTL: IntscnTL,
  FaultStatusInfo_Vehicle: FaultStatusInfo_Vehicle,
  SyncModeSetGear: SyncModeSetGear,
  MoraiTLIndex: MoraiTLIndex,
  ReplayInfo: ReplayInfo,
  Obstacles: Obstacles,
  MoraiSrvResponse: MoraiSrvResponse,
  FaultStatusInfo: FaultStatusInfo,
  EgoVehicleStatusExtended: EgoVehicleStatusExtended,
  EventInfo: EventInfo,
  NpcGhostCmd: NpcGhostCmd,
  FaultStatusInfo_Overall: FaultStatusInfo_Overall,
  FaultStatusInfo_Sensor: FaultStatusInfo_Sensor,
  GhostMessage: GhostMessage,
  SetTrafficLight: SetTrafficLight,
  WaitForTickResponse: WaitForTickResponse,
  GPSMessage: GPSMessage,
  PRStatus: PRStatus,
  MoraiSimProcHandle: MoraiSimProcHandle,
  SkidSteer6wUGVStatus: SkidSteer6wUGVStatus,
  ObjectStatusListExtended: ObjectStatusListExtended,
  EgoDdVehicleStatus: EgoDdVehicleStatus,
  CollisionData: CollisionData,
  MapSpecIndex: MapSpecIndex,
  MultiPlayEventRequest: MultiPlayEventRequest,
  IntersectionStatus: IntersectionStatus,
  ScenarioLoad: ScenarioLoad,
  RobotState: RobotState,
  FaultInjection_Response: FaultInjection_Response,
  SyncModeCmd: SyncModeCmd,
  SyncModeResultResponse: SyncModeResultResponse,
  ShipState: ShipState,
  WaitForTick: WaitForTick,
  PREvent: PREvent,
  CtrlCmd: CtrlCmd,
  SkidSteer6wUGVCtrlCmd: SkidSteer6wUGVCtrlCmd,
  MapSpec: MapSpec,
  RobotOutput: RobotOutput,
  SyncModeCmdResponse: SyncModeCmdResponse,
  TrafficLight: TrafficLight,
  MoraiSimProcStatus: MoraiSimProcStatus,
  VehicleCollisionData: VehicleCollisionData,
  GVDirectCmd: GVDirectCmd,
  NpcGhostInfo: NpcGhostInfo,
  RadarDetections: RadarDetections,
  Transforms: Transforms,
  WheelControl: WheelControl,
  ObjectStatusExtended: ObjectStatusExtended,
  GVStateCmd: GVStateCmd,
  SyncModeCtrlCmd: SyncModeCtrlCmd,
  MultiEgoSetting: MultiEgoSetting,
  EgoVehicleStatus: EgoVehicleStatus,
  ObjectStatusList: ObjectStatusList,
  ManipulatorControl: ManipulatorControl,
  ExternalForce: ExternalForce,
  ERP42Info: ERP42Info,
  VehicleSpec: VehicleSpec,
  GetTrafficLightStatus: GetTrafficLightStatus,
  Obstacle: Obstacle,
  SVADC: SVADC,
  SkateboardCtrlCmd: SkateboardCtrlCmd,
  MultiPlayEventResponse: MultiPlayEventResponse,
  ShipCtrlCmd: ShipCtrlCmd,
};
