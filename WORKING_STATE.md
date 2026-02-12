# Working State (ESP-IDF)

Date: February 11, 2026

Summary of the current ESP-IDF build that is behaving correctly on hardware tests:

- State machine executes full cycle through Homing, Refill, Compression, ReadyToInject, Purge, AntiDrip, Inject, Release, and ConfirmRemoval with the current parameter set.
- Homing tolerates known transient ODrive errors and auto-clears/retries (phase estimate hiccups, CPR/pole mismatch, and transient CL entry errors).
- CAN heartbeat request/response is supported and CAN offline is logged only when the contactor is enabled.
- Endstop safety halts work with inverted top and bottom logic (active-high on trigger).
- AntiDrip move triggers correctly on state entry (no immediate timeout on entry).
- LEDs show solid red when contactor is OFF (cold or cooldown), and use state-based colors when contactor is ON.

Notes:

- This snapshot is intended as the baseline before installing the plunger for full mechanical tests.
