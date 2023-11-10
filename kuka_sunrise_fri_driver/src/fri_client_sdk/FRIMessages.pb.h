/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.8-dev */

#ifndef PB_FRIMESSAGES_PB_H_INCLUDED
#define PB_FRIMESSAGES_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _FRISessionState {
    FRISessionState_IDLE = 0,
    FRISessionState_MONITORING_WAIT = 1,
    FRISessionState_MONITORING_READY = 2,
    FRISessionState_COMMANDING_WAIT = 3,
    FRISessionState_COMMANDING_ACTIVE = 4
} FRISessionState;

typedef enum _FRIConnectionQuality {
    FRIConnectionQuality_POOR = 0,
    FRIConnectionQuality_FAIR = 1,
    FRIConnectionQuality_GOOD = 2,
    FRIConnectionQuality_EXCELLENT = 3
} FRIConnectionQuality;

typedef enum _SafetyState {
    SafetyState_NORMAL_OPERATION = 0,
    SafetyState_SAFETY_STOP_LEVEL_0 = 1,
    SafetyState_SAFETY_STOP_LEVEL_1 = 2,
    SafetyState_SAFETY_STOP_LEVEL_2 = 3
} SafetyState;

typedef enum _OperationMode {
    OperationMode_TEST_MODE_1 = 0,
    OperationMode_TEST_MODE_2 = 1,
    OperationMode_AUTOMATIC_MODE = 2
} OperationMode;

typedef enum _DriveState {
    DriveState_OFF = 0,
    DriveState_TRANSITIONING = 1,
    DriveState_ACTIVE = 2
} DriveState;

typedef enum _ControlMode {
    ControlMode_POSITION_CONTROLMODE = 0,
    ControlMode_CARTESIAN_IMPEDANCE_CONTROLMODE = 1,
    ControlMode_JOINT_IMPEDANCE_CONTROLMODE = 2,
    ControlMode_NO_CONTROLMODE = 3
} ControlMode;

typedef enum _ClientCommandMode {
    ClientCommandMode_NO_COMMAND_MODE = 0,
    ClientCommandMode_POSITION = 1,
    ClientCommandMode_WRENCH = 2,
    ClientCommandMode_TORQUE = 3
} ClientCommandMode;

typedef enum _OverlayType {
    OverlayType_NO_OVERLAY = 0,
    OverlayType_JOINT = 1,
    OverlayType_CARTESIAN = 2
} OverlayType;

typedef enum _FriIOType {
    FriIOType_BOOLEAN = 0,
    FriIOType_DIGITAL = 1,
    FriIOType_ANALOG = 2
} FriIOType;

typedef enum _FriIODirection {
    FriIODirection_INPUT = 0,
    FriIODirection_OUTPUT = 1
} FriIODirection;

/* Struct definitions */
typedef struct _JointValues {
    pb_callback_t value;
} JointValues;

typedef struct _TimeStamp {
    uint32_t sec;
    uint32_t nanosec;
} TimeStamp;

typedef struct _CartesianVector {
    pb_size_t element_count;
    double element[6];
} CartesianVector;

typedef struct _Checksum {
    bool has_crc32;
    int32_t crc32;
} Checksum;

typedef struct _Transformation {
    char name[64];
    pb_size_t matrix_count;
    double matrix[12];
    bool has_timestamp;
    TimeStamp timestamp;
} Transformation;

typedef struct _FriIOValue {
    char name[64];
    FriIOType type;
    FriIODirection direction;
    bool has_digitalValue;
    uint64_t digitalValue;
    bool has_analogValue;
    double analogValue;
} FriIOValue;

typedef struct _MessageHeader {
    uint32_t messageIdentifier;
    uint32_t sequenceCounter;
    uint32_t reflectedSequenceCounter;
} MessageHeader;

typedef struct _ConnectionInfo {
    FRISessionState sessionState;
    FRIConnectionQuality quality;
    bool has_sendPeriod;
    uint32_t sendPeriod;
    bool has_receiveMultiplier;
    uint32_t receiveMultiplier;
} ConnectionInfo;

typedef struct _RobotInfo {
    bool has_numberOfJoints;
    int32_t numberOfJoints;
    bool has_safetyState;
    SafetyState safetyState;
    pb_callback_t driveState;
    bool has_operationMode;
    OperationMode operationMode;
    bool has_controlMode;
    ControlMode controlMode;
} RobotInfo;

typedef struct _MessageMonitorData {
    bool has_measuredJointPosition;
    JointValues measuredJointPosition;
    bool has_measuredTorque;
    JointValues measuredTorque;
    bool has_commandedJointPosition;
    JointValues commandedJointPosition;
    bool has_commandedTorque;
    JointValues commandedTorque;
    bool has_externalTorque;
    JointValues externalTorque;
    pb_size_t readIORequest_count;
    FriIOValue readIORequest[10];
    bool has_timestamp;
    TimeStamp timestamp;
} MessageMonitorData;

typedef struct _MessageIpoData {
    bool has_jointPosition;
    JointValues jointPosition;
    bool has_clientCommandMode;
    ClientCommandMode clientCommandMode;
    bool has_overlayType;
    OverlayType overlayType;
    bool has_trackingPerformance;
    double trackingPerformance;
} MessageIpoData;

typedef struct _MessageCommandData {
    bool has_jointPosition;
    JointValues jointPosition;
    bool has_cartesianWrenchFeedForward;
    CartesianVector cartesianWrenchFeedForward;
    bool has_jointTorque;
    JointValues jointTorque;
    pb_size_t commandedTransformations_count;
    Transformation commandedTransformations[5];
    pb_size_t writeIORequest_count;
    FriIOValue writeIORequest[10];
} MessageCommandData;

typedef struct _MessageEndOf {
    bool has_messageLength;
    int32_t messageLength;
    bool has_messageChecksum;
    Checksum messageChecksum;
} MessageEndOf;

typedef struct _FRIMonitoringMessage {
    MessageHeader header;
    bool has_robotInfo;
    RobotInfo robotInfo;
    bool has_monitorData;
    MessageMonitorData monitorData;
    bool has_connectionInfo;
    ConnectionInfo connectionInfo;
    bool has_ipoData;
    MessageIpoData ipoData;
    pb_size_t requestedTransformations_count;
    Transformation requestedTransformations[5];
    bool has_endOfMessageData;
    MessageEndOf endOfMessageData;
} FRIMonitoringMessage;

typedef struct _FRICommandMessage {
    MessageHeader header;
    bool has_commandData;
    MessageCommandData commandData;
    bool has_endOfMessageData;
    MessageEndOf endOfMessageData;
} FRICommandMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _FRISessionState_MIN FRISessionState_IDLE
#define _FRISessionState_MAX FRISessionState_COMMANDING_ACTIVE
#define _FRISessionState_ARRAYSIZE ((FRISessionState)(FRISessionState_COMMANDING_ACTIVE+1))

#define _FRIConnectionQuality_MIN FRIConnectionQuality_POOR
#define _FRIConnectionQuality_MAX FRIConnectionQuality_EXCELLENT
#define _FRIConnectionQuality_ARRAYSIZE ((FRIConnectionQuality)(FRIConnectionQuality_EXCELLENT+1))

#define _SafetyState_MIN SafetyState_NORMAL_OPERATION
#define _SafetyState_MAX SafetyState_SAFETY_STOP_LEVEL_2
#define _SafetyState_ARRAYSIZE ((SafetyState)(SafetyState_SAFETY_STOP_LEVEL_2+1))

#define _OperationMode_MIN OperationMode_TEST_MODE_1
#define _OperationMode_MAX OperationMode_AUTOMATIC_MODE
#define _OperationMode_ARRAYSIZE ((OperationMode)(OperationMode_AUTOMATIC_MODE+1))

#define _DriveState_MIN DriveState_OFF
#define _DriveState_MAX DriveState_ACTIVE
#define _DriveState_ARRAYSIZE ((DriveState)(DriveState_ACTIVE+1))

#define _ControlMode_MIN ControlMode_POSITION_CONTROLMODE
#define _ControlMode_MAX ControlMode_NO_CONTROLMODE
#define _ControlMode_ARRAYSIZE ((ControlMode)(ControlMode_NO_CONTROLMODE+1))

#define _ClientCommandMode_MIN ClientCommandMode_NO_COMMAND_MODE
#define _ClientCommandMode_MAX ClientCommandMode_TORQUE
#define _ClientCommandMode_ARRAYSIZE ((ClientCommandMode)(ClientCommandMode_TORQUE+1))

#define _OverlayType_MIN OverlayType_NO_OVERLAY
#define _OverlayType_MAX OverlayType_CARTESIAN
#define _OverlayType_ARRAYSIZE ((OverlayType)(OverlayType_CARTESIAN+1))

#define _FriIOType_MIN FriIOType_BOOLEAN
#define _FriIOType_MAX FriIOType_ANALOG
#define _FriIOType_ARRAYSIZE ((FriIOType)(FriIOType_ANALOG+1))

#define _FriIODirection_MIN FriIODirection_INPUT
#define _FriIODirection_MAX FriIODirection_OUTPUT
#define _FriIODirection_ARRAYSIZE ((FriIODirection)(FriIODirection_OUTPUT+1))






#define FriIOValue_type_ENUMTYPE FriIOType
#define FriIOValue_direction_ENUMTYPE FriIODirection


#define ConnectionInfo_sessionState_ENUMTYPE FRISessionState
#define ConnectionInfo_quality_ENUMTYPE FRIConnectionQuality

#define RobotInfo_safetyState_ENUMTYPE SafetyState
#define RobotInfo_driveState_ENUMTYPE DriveState
#define RobotInfo_operationMode_ENUMTYPE OperationMode
#define RobotInfo_controlMode_ENUMTYPE ControlMode


#define MessageIpoData_clientCommandMode_ENUMTYPE ClientCommandMode
#define MessageIpoData_overlayType_ENUMTYPE OverlayType






/* Initializer values for message structs */
#define JointValues_init_default                 {{{NULL}, NULL}}
#define TimeStamp_init_default                   {0, 0}
#define CartesianVector_init_default             {0, {0, 0, 0, 0, 0, 0}}
#define Checksum_init_default                    {false, 0}
#define Transformation_init_default              {"", 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, false, TimeStamp_init_default}
#define FriIOValue_init_default                  {"", _FriIOType_MIN, _FriIODirection_MIN, false, 0, false, 0}
#define MessageHeader_init_default               {0, 0, 0}
#define ConnectionInfo_init_default              {_FRISessionState_MIN, _FRIConnectionQuality_MIN, false, 0, false, 0}
#define RobotInfo_init_default                   {false, 0, false, _SafetyState_MIN, {{NULL}, NULL}, false, _OperationMode_MIN, false, _ControlMode_MIN}
#define MessageMonitorData_init_default          {false, JointValues_init_default, false, JointValues_init_default, false, JointValues_init_default, false, JointValues_init_default, false, JointValues_init_default, 0, {FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default}, false, TimeStamp_init_default}
#define MessageIpoData_init_default              {false, JointValues_init_default, false, _ClientCommandMode_MIN, false, _OverlayType_MIN, false, 0}
#define MessageCommandData_init_default          {false, JointValues_init_default, false, CartesianVector_init_default, false, JointValues_init_default, 0, {Transformation_init_default, Transformation_init_default, Transformation_init_default, Transformation_init_default, Transformation_init_default}, 0, {FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default, FriIOValue_init_default}}
#define MessageEndOf_init_default                {false, 0, false, Checksum_init_default}
#define FRIMonitoringMessage_init_default        {MessageHeader_init_default, false, RobotInfo_init_default, false, MessageMonitorData_init_default, false, ConnectionInfo_init_default, false, MessageIpoData_init_default, 0, {Transformation_init_default, Transformation_init_default, Transformation_init_default, Transformation_init_default, Transformation_init_default}, false, MessageEndOf_init_default}
#define FRICommandMessage_init_default           {MessageHeader_init_default, false, MessageCommandData_init_default, false, MessageEndOf_init_default}
#define JointValues_init_zero                    {{{NULL}, NULL}}
#define TimeStamp_init_zero                      {0, 0}
#define CartesianVector_init_zero                {0, {0, 0, 0, 0, 0, 0}}
#define Checksum_init_zero                       {false, 0}
#define Transformation_init_zero                 {"", 0, {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, false, TimeStamp_init_zero}
#define FriIOValue_init_zero                     {"", _FriIOType_MIN, _FriIODirection_MIN, false, 0, false, 0}
#define MessageHeader_init_zero                  {0, 0, 0}
#define ConnectionInfo_init_zero                 {_FRISessionState_MIN, _FRIConnectionQuality_MIN, false, 0, false, 0}
#define RobotInfo_init_zero                      {false, 0, false, _SafetyState_MIN, {{NULL}, NULL}, false, _OperationMode_MIN, false, _ControlMode_MIN}
#define MessageMonitorData_init_zero             {false, JointValues_init_zero, false, JointValues_init_zero, false, JointValues_init_zero, false, JointValues_init_zero, false, JointValues_init_zero, 0, {FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero}, false, TimeStamp_init_zero}
#define MessageIpoData_init_zero                 {false, JointValues_init_zero, false, _ClientCommandMode_MIN, false, _OverlayType_MIN, false, 0}
#define MessageCommandData_init_zero             {false, JointValues_init_zero, false, CartesianVector_init_zero, false, JointValues_init_zero, 0, {Transformation_init_zero, Transformation_init_zero, Transformation_init_zero, Transformation_init_zero, Transformation_init_zero}, 0, {FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero, FriIOValue_init_zero}}
#define MessageEndOf_init_zero                   {false, 0, false, Checksum_init_zero}
#define FRIMonitoringMessage_init_zero           {MessageHeader_init_zero, false, RobotInfo_init_zero, false, MessageMonitorData_init_zero, false, ConnectionInfo_init_zero, false, MessageIpoData_init_zero, 0, {Transformation_init_zero, Transformation_init_zero, Transformation_init_zero, Transformation_init_zero, Transformation_init_zero}, false, MessageEndOf_init_zero}
#define FRICommandMessage_init_zero              {MessageHeader_init_zero, false, MessageCommandData_init_zero, false, MessageEndOf_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define JointValues_value_tag                    1
#define TimeStamp_sec_tag                        1
#define TimeStamp_nanosec_tag                    2
#define CartesianVector_element_tag              1
#define Checksum_crc32_tag                       1
#define Transformation_name_tag                  1
#define Transformation_matrix_tag                2
#define Transformation_timestamp_tag             3
#define FriIOValue_name_tag                      1
#define FriIOValue_type_tag                      2
#define FriIOValue_direction_tag                 3
#define FriIOValue_digitalValue_tag              4
#define FriIOValue_analogValue_tag               5
#define MessageHeader_messageIdentifier_tag      1
#define MessageHeader_sequenceCounter_tag        2
#define MessageHeader_reflectedSequenceCounter_tag 3
#define ConnectionInfo_sessionState_tag          1
#define ConnectionInfo_quality_tag               2
#define ConnectionInfo_sendPeriod_tag            3
#define ConnectionInfo_receiveMultiplier_tag     4
#define RobotInfo_numberOfJoints_tag             1
#define RobotInfo_safetyState_tag                2
#define RobotInfo_driveState_tag                 5
#define RobotInfo_operationMode_tag              6
#define RobotInfo_controlMode_tag                7
#define MessageMonitorData_measuredJointPosition_tag 1
#define MessageMonitorData_measuredTorque_tag    2
#define MessageMonitorData_commandedJointPosition_tag 3
#define MessageMonitorData_commandedTorque_tag   4
#define MessageMonitorData_externalTorque_tag    5
#define MessageMonitorData_readIORequest_tag     8
#define MessageMonitorData_timestamp_tag         15
#define MessageIpoData_jointPosition_tag         1
#define MessageIpoData_clientCommandMode_tag     10
#define MessageIpoData_overlayType_tag           11
#define MessageIpoData_trackingPerformance_tag   12
#define MessageCommandData_jointPosition_tag     1
#define MessageCommandData_cartesianWrenchFeedForward_tag 2
#define MessageCommandData_jointTorque_tag       3
#define MessageCommandData_commandedTransformations_tag 4
#define MessageCommandData_writeIORequest_tag    5
#define MessageEndOf_messageLength_tag           1
#define MessageEndOf_messageChecksum_tag         2
#define FRIMonitoringMessage_header_tag          1
#define FRIMonitoringMessage_robotInfo_tag       2
#define FRIMonitoringMessage_monitorData_tag     3
#define FRIMonitoringMessage_connectionInfo_tag  4
#define FRIMonitoringMessage_ipoData_tag         5
#define FRIMonitoringMessage_requestedTransformations_tag 6
#define FRIMonitoringMessage_endOfMessageData_tag 15
#define FRICommandMessage_header_tag             1
#define FRICommandMessage_commandData_tag        2
#define FRICommandMessage_endOfMessageData_tag   15

/* Struct field encoding specification for nanopb */
#define JointValues_FIELDLIST(X, a) \
X(a, CALLBACK, REPEATED, DOUBLE,   value,             1)
#define JointValues_CALLBACK pb_default_field_callback
#define JointValues_DEFAULT NULL

#define TimeStamp_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   sec,               1) \
X(a, STATIC,   REQUIRED, UINT32,   nanosec,           2)
#define TimeStamp_CALLBACK NULL
#define TimeStamp_DEFAULT NULL

#define CartesianVector_FIELDLIST(X, a) \
X(a, STATIC,   REPEATED, DOUBLE,   element,           1)
#define CartesianVector_CALLBACK NULL
#define CartesianVector_DEFAULT NULL

#define Checksum_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, INT32,    crc32,             1)
#define Checksum_CALLBACK NULL
#define Checksum_DEFAULT NULL

#define Transformation_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, STRING,   name,              1) \
X(a, STATIC,   REPEATED, DOUBLE,   matrix,            2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  timestamp,         3)
#define Transformation_CALLBACK NULL
#define Transformation_DEFAULT NULL
#define Transformation_timestamp_MSGTYPE TimeStamp

#define FriIOValue_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, STRING,   name,              1) \
X(a, STATIC,   REQUIRED, UENUM,    type,              2) \
X(a, STATIC,   REQUIRED, UENUM,    direction,         3) \
X(a, STATIC,   OPTIONAL, UINT64,   digitalValue,      4) \
X(a, STATIC,   OPTIONAL, DOUBLE,   analogValue,       5)
#define FriIOValue_CALLBACK NULL
#define FriIOValue_DEFAULT NULL

#define MessageHeader_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UINT32,   messageIdentifier,   1) \
X(a, STATIC,   REQUIRED, UINT32,   sequenceCounter,   2) \
X(a, STATIC,   REQUIRED, UINT32,   reflectedSequenceCounter,   3)
#define MessageHeader_CALLBACK NULL
#define MessageHeader_DEFAULT NULL

#define ConnectionInfo_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, UENUM,    sessionState,      1) \
X(a, STATIC,   REQUIRED, UENUM,    quality,           2) \
X(a, STATIC,   OPTIONAL, UINT32,   sendPeriod,        3) \
X(a, STATIC,   OPTIONAL, UINT32,   receiveMultiplier,   4)
#define ConnectionInfo_CALLBACK NULL
#define ConnectionInfo_DEFAULT NULL

#define RobotInfo_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, INT32,    numberOfJoints,    1) \
X(a, STATIC,   OPTIONAL, UENUM,    safetyState,       2) \
X(a, CALLBACK, REPEATED, UENUM,    driveState,        5) \
X(a, STATIC,   OPTIONAL, UENUM,    operationMode,     6) \
X(a, STATIC,   OPTIONAL, UENUM,    controlMode,       7)
#define RobotInfo_CALLBACK pb_default_field_callback
#define RobotInfo_DEFAULT NULL

#define MessageMonitorData_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  measuredJointPosition,   1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  measuredTorque,    2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  commandedJointPosition,   3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  commandedTorque,   4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  externalTorque,    5) \
X(a, STATIC,   REPEATED, MESSAGE,  readIORequest,     8) \
X(a, STATIC,   OPTIONAL, MESSAGE,  timestamp,        15)
#define MessageMonitorData_CALLBACK NULL
#define MessageMonitorData_DEFAULT NULL
#define MessageMonitorData_measuredJointPosition_MSGTYPE JointValues
#define MessageMonitorData_measuredTorque_MSGTYPE JointValues
#define MessageMonitorData_commandedJointPosition_MSGTYPE JointValues
#define MessageMonitorData_commandedTorque_MSGTYPE JointValues
#define MessageMonitorData_externalTorque_MSGTYPE JointValues
#define MessageMonitorData_readIORequest_MSGTYPE FriIOValue
#define MessageMonitorData_timestamp_MSGTYPE TimeStamp

#define MessageIpoData_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  jointPosition,     1) \
X(a, STATIC,   OPTIONAL, UENUM,    clientCommandMode,  10) \
X(a, STATIC,   OPTIONAL, UENUM,    overlayType,      11) \
X(a, STATIC,   OPTIONAL, DOUBLE,   trackingPerformance,  12)
#define MessageIpoData_CALLBACK NULL
#define MessageIpoData_DEFAULT NULL
#define MessageIpoData_jointPosition_MSGTYPE JointValues

#define MessageCommandData_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  jointPosition,     1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  cartesianWrenchFeedForward,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  jointTorque,       3) \
X(a, STATIC,   REPEATED, MESSAGE,  commandedTransformations,   4) \
X(a, STATIC,   REPEATED, MESSAGE,  writeIORequest,    5)
#define MessageCommandData_CALLBACK NULL
#define MessageCommandData_DEFAULT NULL
#define MessageCommandData_jointPosition_MSGTYPE JointValues
#define MessageCommandData_cartesianWrenchFeedForward_MSGTYPE CartesianVector
#define MessageCommandData_jointTorque_MSGTYPE JointValues
#define MessageCommandData_commandedTransformations_MSGTYPE Transformation
#define MessageCommandData_writeIORequest_MSGTYPE FriIOValue

#define MessageEndOf_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, INT32,    messageLength,     1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  messageChecksum,   2)
#define MessageEndOf_CALLBACK NULL
#define MessageEndOf_DEFAULT NULL
#define MessageEndOf_messageChecksum_MSGTYPE Checksum

#define FRIMonitoringMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  robotInfo,         2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  monitorData,       3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  connectionInfo,    4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  ipoData,           5) \
X(a, STATIC,   REPEATED, MESSAGE,  requestedTransformations,   6) \
X(a, STATIC,   OPTIONAL, MESSAGE,  endOfMessageData,  15)
#define FRIMonitoringMessage_CALLBACK NULL
#define FRIMonitoringMessage_DEFAULT NULL
#define FRIMonitoringMessage_header_MSGTYPE MessageHeader
#define FRIMonitoringMessage_robotInfo_MSGTYPE RobotInfo
#define FRIMonitoringMessage_monitorData_MSGTYPE MessageMonitorData
#define FRIMonitoringMessage_connectionInfo_MSGTYPE ConnectionInfo
#define FRIMonitoringMessage_ipoData_MSGTYPE MessageIpoData
#define FRIMonitoringMessage_requestedTransformations_MSGTYPE Transformation
#define FRIMonitoringMessage_endOfMessageData_MSGTYPE MessageEndOf

#define FRICommandMessage_FIELDLIST(X, a) \
X(a, STATIC,   REQUIRED, MESSAGE,  header,            1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  commandData,       2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  endOfMessageData,  15)
#define FRICommandMessage_CALLBACK NULL
#define FRICommandMessage_DEFAULT NULL
#define FRICommandMessage_header_MSGTYPE MessageHeader
#define FRICommandMessage_commandData_MSGTYPE MessageCommandData
#define FRICommandMessage_endOfMessageData_MSGTYPE MessageEndOf

extern const pb_msgdesc_t JointValues_msg;
extern const pb_msgdesc_t TimeStamp_msg;
extern const pb_msgdesc_t CartesianVector_msg;
extern const pb_msgdesc_t Checksum_msg;
extern const pb_msgdesc_t Transformation_msg;
extern const pb_msgdesc_t FriIOValue_msg;
extern const pb_msgdesc_t MessageHeader_msg;
extern const pb_msgdesc_t ConnectionInfo_msg;
extern const pb_msgdesc_t RobotInfo_msg;
extern const pb_msgdesc_t MessageMonitorData_msg;
extern const pb_msgdesc_t MessageIpoData_msg;
extern const pb_msgdesc_t MessageCommandData_msg;
extern const pb_msgdesc_t MessageEndOf_msg;
extern const pb_msgdesc_t FRIMonitoringMessage_msg;
extern const pb_msgdesc_t FRICommandMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define JointValues_fields &JointValues_msg
#define TimeStamp_fields &TimeStamp_msg
#define CartesianVector_fields &CartesianVector_msg
#define Checksum_fields &Checksum_msg
#define Transformation_fields &Transformation_msg
#define FriIOValue_fields &FriIOValue_msg
#define MessageHeader_fields &MessageHeader_msg
#define ConnectionInfo_fields &ConnectionInfo_msg
#define RobotInfo_fields &RobotInfo_msg
#define MessageMonitorData_fields &MessageMonitorData_msg
#define MessageIpoData_fields &MessageIpoData_msg
#define MessageCommandData_fields &MessageCommandData_msg
#define MessageEndOf_fields &MessageEndOf_msg
#define FRIMonitoringMessage_fields &FRIMonitoringMessage_msg
#define FRICommandMessage_fields &FRICommandMessage_msg

/* Maximum encoded size of messages (where known) */
/* JointValues_size depends on runtime parameters */
/* RobotInfo_size depends on runtime parameters */
/* MessageMonitorData_size depends on runtime parameters */
/* MessageIpoData_size depends on runtime parameters */
/* MessageCommandData_size depends on runtime parameters */
/* FRIMonitoringMessage_size depends on runtime parameters */
/* FRICommandMessage_size depends on runtime parameters */
#define CartesianVector_size                     54
#define Checksum_size                            11
#define ConnectionInfo_size                      16
#define FriIOValue_size                          89
#define MessageEndOf_size                        24
#define MessageHeader_size                       18
#define TimeStamp_size                           12
#define Transformation_size                      187

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif