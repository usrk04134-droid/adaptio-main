# Diagram

```plantuml

class "WebHmiServer"
class "OperationDataImpl"
class "ServiceModeManagerImpl"
class "ServiceModeKinematicsControl"
class "ServiceModeTracking"
interface "OperationData"
interface "kinematics::KinematicsClientObserver"
interface "slice_translator::SliceObserver"
interface "kinematics::KinematicsClient"
interface "tracking::TrackingManager"
interface "scanner::ScannerClient"

ServiceModeManagerImpl <-- WebHmiServer
OperationData <|-- OperationDataImpl
kinematics::KinematicsClientObserver <|-- WebHmiServer
slice_translator::SliceObserver <|-- WebHmiServer

ServiceModeKinematicsControl <-- ServiceModeManagerImpl
ServiceModeTracking <-- ServiceModeManagerImpl

tracking::TrackingManager <-- ServiceModeTracking
kinematics::KinematicsClient <-- ServiceModeKinematicsControl

```
