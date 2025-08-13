# Diagram

```plantuml
hide footbox

' Threads / event loops
participant "Main" as Main
participant "Application (EventLoop)" as App
participant "Controller Messenger (EventLoop)" as Ctrl
participant "Scanner (EventLoop)" as Scan

' ZEVS timer threads (one per event loop)
participant "TimerThread[Application]" as TimerApp
participant "TimerThread[Controller]" as TimerCtrl
participant "TimerThread[Scanner]" as TimerScan

== Startup ==
Main -> Ctrl: StartThread("Controller Messenger")
activate Ctrl
Ctrl -> Ctrl: CreateEventLoop("Controller Messenger")
Ctrl -> Ctrl: Bind PAIR\n- inproc://adaptio/management\n- inproc://adaptio/kinematics\n- inproc://adaptio/weld-system
Ctrl -> TimerCtrl: Start timer thread\n(timer endpoint: inproc://Controller Messenger_timer)

Main -> Scan: StartThread("Scanner")
activate Scan
Scan -> Scan: CreateEventLoop("Scanner")
Scan -> Scan: Bind PAIR\n- inproc://adaptio/scanner\nRegister Serve:\n- scanner::Start/Stop/Update\n- ImageLoggingUpdate/FlushImageBuffer
Scan -> TimerScan: Start timer thread\n(timer endpoint: inproc://Scanner_timer)

Main -> App: Run("Application")
activate App
App -> App: CreateEventLoop("Application")
App -> App: Connect PAIR\n- inproc://adaptio/kinematics\n- inproc://adaptio/scanner\n- inproc://adaptio/weld-system\nApp -> App: Management PAIR connect\n- inproc://adaptio/management
App -> App: WebHMI SUB/PUB bind
App -> TimerApp: Start timer thread\n(timer endpoint: inproc://Application_timer)
App -> App: event_loop.Run() (blocks main thread)

== Scanner interaction ==
App -> Scan: scanner::Start
Scan --> App: scanner::StartRsp(success=ok)

loop periodic (input interval)
  TimerScan -> Scan: ADAPTIO_TIMER (timeout)
  Scan -> Scan: OnTimeout()\ncore_scanner.Update()
  Scan -> App: scanner::SliceData
end

== Controller interaction ==
App -> Ctrl: kinematics::GetSlidesPosition
Ctrl --> App: kinematics::GetSlidesPositionRsp
App -> Ctrl: kinematics::SetSlidesPosition

loop controller cycle (cycle_time_ms)
  TimerCtrl -> Ctrl: ADAPTIO_TIMER (timeout)
  Ctrl -> Ctrl: OnTimeout()\n- RetrieveInputs\n- ManagementClient.Update\n- WriteOutputs
end

== Management and state ==
Ctrl -> Ctrl: OnAdaptioInput/TrackInput\n=> ManagementClient events
Ctrl -> Ctrl: OnPowerSourceInput(n)\n=> WeldSystemServer

== Shutdown ==
Main -> App: zevs::ExitEventLoop("Application")
Main -> Ctrl: zevs::ExitEventLoop("Controller Messenger")
Main -> Scan: zevs::ExitEventLoop("Scanner")
App --> Main: event loop exits
Ctrl --> Main: event loop exits
Scan --> Main: event loop exits
TimerApp --> App: thread stops
TimerCtrl --> Ctrl: thread stops
TimerScan --> Scan: thread stops

note over App,Ctrl
- ZEVS creates one TimerThread per EventLoop\n- TimerThread publishes ADAPTIO_TIMER to event loop via inproc PUB/SUB\n- TimerBackend dispatches callbacks in the owning event loop thread
end note
```