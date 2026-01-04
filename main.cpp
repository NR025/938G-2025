#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// motor groups
pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({18, 19, 20}, pros::MotorGearset::blue);

pros::Motor intake(9, pros::MotorGearset::blue); 
pros::Motor hood(-5, pros::MotorGearset::blue); 

// Inertial Sensor on port 10
pros::Imu imu(10);

// Drivetrain and Controller Settings (Keeping your LemLib settings)
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 10, lemlib::Omniwheel::NEW_325, 480, 8);
lemlib::ControllerSettings linearController(10, 0, 3, 3, 1, 100, 3, 500, 20);
lemlib::ControllerSettings angularController(2, 0, 10, 3, 1, 100, 3, 500, 0);
lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);
lemlib::ExpoDriveCurve throttleCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve steerCurve(3, 10, 1.019);
lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// 1. Add this at the top with your other variables
bool isUnjamming = false; 

// 2. Update your Anti-Jam Task
void antiJamTask(void* param) {
    while (true) {
        // Trigger if we want it to move (>0) but it's stuck (<20)
        if (intake.get_target_velocity() > 0 && intake.get_actual_velocity() < 20) {
            
            pros::delay(150); // Small buffer to ensure it's a real jam

            // Re-check: still jammed?
            if (intake.get_target_velocity() > 0 && intake.get_actual_velocity() < 20) {
                isUnjamming = true; // LOCK: opcontrol will now ignore intake buttons
                
                intake.move_velocity(-600);
                hood.move_velocity(-600);
                pros::delay(300); // Back up for half a second
                
                isUnjamming = false; // UNLOCK: give control back to driver
            }
        }
        pros::delay(20);
    }
} 

void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    pros::Task antiJam(antiJamTask);

    // Position Logging task
    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x);
            pros::lcd::print(1, "Y: %f", chassis.getPose().y);
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta);
            pros::delay(50);
        }
    });
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

// --- MAIN OPCONTROL ---
void opcontrol() {
    // Set brake mode once at the start
    hood.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    while (true) {
        // --- INTAKE & HOOD CONTROL ---
        // We only process these buttons if the Anti-Jam isn't currently reversing
        if (!isUnjamming) {
            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                // Regular Intake + Hood
                intake.move_velocity(600);
                hood.move_velocity(600);
            } 
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                // Intake only (your specific R2 request)
                intake.move_velocity(600);
                hood.move_velocity(0);
            } 
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
                // Full Reverse
                intake.move_velocity(-600);
                hood.move_velocity(-600);
            } 
            else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
                // Slow Sort/Outtake
                intake.move_velocity(200);
                hood.move_velocity(200);
            } 
            else {
                // Stop if no buttons are pressed
                intake.move_velocity(0);
                hood.move_velocity(0);
            }
        }

        // --- DRIVE CONTROL ---
        // This is OUTSIDE the 'if(!isUnjamming)' so you can always drive, 
        // even while the robot is clearing a jam!
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        //  Deadzone 
        if (std::abs(leftY) < 5) leftY = 0;
        if (std::abs(rightX) < 5) rightX = 0;

        // Apply drive to LemLib chassis
        chassis.arcade(leftY, rightX);

        // 10ms delay for standard PROS loop timing
        pros::delay(10);
    }
}