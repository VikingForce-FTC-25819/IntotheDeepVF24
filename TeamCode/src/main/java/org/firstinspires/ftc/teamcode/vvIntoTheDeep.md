## Vibranium Viking Team Info

Creation of documentation regarding how our code flows

Hardware Abstraction and proper programming structure is critical to ensure reliability and improve troubleshooting

All devices start with configuration and methods in the vvHardware.java class
Using 'robot.' (or whatever is declared in the respective class, for this class call) can call any hardware from that class into the active class for reuse and consistency

For Trajectory Sequences we will add vvHardwareRR to call the methods for trajectory building and any sequence automation.

Packages are used to keep things organized
 - Auton, Concept, Core, & TeleOp are packages our team will work within
 - drive, trajectorysequence, and util are used for Core extensions and tuning
 - Core is for any classes we intent to call for methods and actions
 - Concept will be any troubleshooting or learning classes

Tuning is required for every robot due to differences in mass, structure, and controls
