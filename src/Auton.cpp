#include "config.h"
#include "pid.h"
#include "myFunctions.h"


void bothMove(int vel){
    left.move_velocity(vel);
    right.move_velocity(vel);
}
void Delay(int time){
    for (int i = 0;i < time/20; i++){
        pros::delay(20);
    }
    pros::delay(time%20);
}
std::tuple<bool> Delay(int time, bool armIntake){
    for (int i = 0;i < time/20; i++){
        if (armIntake){
            if (armSensor.get()<50){
                chain.move(0);
            }
        }
        pros::delay(20);
    }
    pros::delay(time%20);
    return std::make_tuple(armIntake);
}

void scuffed(){
    chain.move(-600);
    arm.move(127);
    bothMove(-400);
    pros::delay(400);
    arm.move(0);
    chain.move(0);
    pros::delay(300);
    clamp.set_value(1);
    bothMove(-200);
    pros::delay(400);
    PidTurnTo(125);
    pros::delay(100);
    intake.move(600);
    chain.move(600);
    bothMove(300);
    pros::delay(400);
    PidTurnTo(80 );
    pros::delay(100);
    bothMove(200);
    pros::delay(300);
    PidTurnTo(80 );
    pros::delay(100);
    bothMove(200);
    pros::delay(400);
    bothMove(0);
    PidTurnTo(320 );
    bothMove(220);
    pros::delay(620);
    PidTurnTo(280);
    bothMove(400);
    pros::delay(700);
    bothMove(200);
    pros::delay(700);
    bothMove(0);
    PidTurnTo(180 );
    bothMove(200);
    pros::delay(700);
    bothMove(0);
}

void mogoRush(){
    brakehold();
    bothMove(-600);
    Delay(480);
    bothMove(100);
    Delay(20);
    bothMove(0);
    intake.move(600);
    chain.move(600);
    PidTurnRightTo(90);
    bothMove(-600);
    Delay(140);
    bothMove(100);
    Delay(20);
    bothMove(0);
    PidTurnLeftTo(0);
    bothMove(-600);
    Delay(150);
    bothMove(600);
    Delay(50);
    clamp.set_value(1);
    Delay(450);
    bothMove(0);
    PidTurnTo(-65);
    intake.move(0);
    bothMove(600);
    Delay(800);
    intake.move(80);
    bothMove(-100);
    Delay(40);
    PidTurnTo(-100);
    intake.move(-600);
    chain.move(0);
    Delay(600);
    PidTurnTo(-65);
    intake.move(600);
    arm.move(600);
    Delay(100);
    bothMove(600);
    Delay(500);
    bothMove(-600);
    Delay(480);
    PidTurnTo(0);
    arm.move(-600);
    bothMove(300);
    Delay(600);
    bothMove(0);
    Delay(200);
    bothMove(-300);
    Delay(700);
    chain.move(600);
    arm.move(0);
    bothMove(0);





}

void ringRushRed(){
    bool armIntake = true;
    brakehold(); //Move to wall stake, facing rings
    chain.move(-600);
    bothMove(600);
    Delay(410);
    chain.move(0);
    Delay(200);
    bothMove(-100);
    arm.move(0);
    Delay(20);
    bothMove(0);
    PidTurnLeftTo(90);
    intake.move(600);
    chain.move(600);

    bothMove(200); //intake rings
    Delay(1200);
    bothMove(0);
    Delay(150);
    
    PidTurnTo(-55,armIntake); //get mogo
    chain.move(0);
    Delay(80);
    bothMove(-300);
    Delay(600);
    for (int i = 0; i<2;i++){
        right.move_velocity(0);
        left.move_velocity(-400);
        Delay(120);
        left.move_velocity(0);
        right.move_velocity(-400);
        Delay(120);
    }
    arm.move(-127);
    bothMove(0);
    clamp.set_value(1);
    Delay(100);
    arm.move(0);
    PidTurnTo(-100); //Get Center Ring
    chain.move(600);
    Delay(100);
    bothMove(600);
    Delay(300);
    Delay(180);
    bothMove(-100);
    Delay(40);
    chain.move(0);
    PidTurnTo(160);
    chain.move(600);
    bothMove(400);
    Delay(700);
    bothMove(0);
    PidTurnTo(78);
    chain.move(600);
    bothMove(600);
    Delay(800);
    intake.move(0);
    bothMove(-100);
    Delay(40);
    chain.move(0);
    PidTurnTo(160);
    intake.move(-600);
    Delay(600);
    chain.move(600);
    PidTurnTo(78);
    intake.move(600);
    bothMove(600);
    Delay(500);
    bothMove(-600);
    Delay(500);
    PidTurnTo(0);
    bothMove(600);
    Delay(600);
    arm.move(600);
    bothMove(0);
    

}

void soloWPRed() {
    // release
    chain.move(-127);

    //ring side

    //get stake    
    bothMove(-600);
    Delay(120);
    chain.move(0);
    Delay(350);
    bothMove(0);
    brakehold();
    Delay(30);
    for (int i = 0; i<3;i++){
        right.move_velocity(0);
        left.move_velocity(-400);
        Delay(120);
        left.move_velocity(0);
        right.move_velocity(-400);
        Delay(120);
    }
    bothMove(0);
    clamp.set_value(1);
    Delay(30);

    intake.move(127);
    chain.move(127);
    
    // get single ring
    PidTurnTo(90);
    bothMove(0);
    Delay(40);
    bothMove(600);
    Delay(200);
    bothMove(400);
    Delay(300);

    // go to other side
    PidTurnTo(320);
    bothMove(0);
    Delay(40);
    bothMove(600);
    Delay(520); //decreased from 650 to 540 backing before turn
    PidTurnTo(270);
    brakehold();
    bothMove(0);
    Delay(50);
    intake.move(127);
    bothMove(600);
    Delay(400); 
    bothMove(300);
    Delay(350); 
    chain.move(0);
    intake.move(0);
    PidTurnTo(-50);
    intake.move(-600);
    Delay(600);
    PidTurnTo(270);
    chain.move(600);
    intake.move(600);
    delay(100);
    bothMove(600);
    Delay(400);
    //Change from 300 to 350
    // Part 2 starts

    // get stake
    chain.move(0);
    clamp.set_value(0);
    // Part 2 startes
    bothMove(0);
    Delay(150);
    bothMove(0);
    PidTurnTo(30);
    bothMove(-600);
    Delay(400);
    bothMove(0);
    brakehold();
    Delay(50);

    for (int i = 0; i<3;i++){
        right.move_velocity(0);
        left.move_velocity(-400);
        Delay(120);
        left.move_velocity(0);
        right.move_velocity(-400);
        Delay(120);
    }
    bothMove(0);
    clamp.set_value(1);
    Delay(20);

    // get ring
    PidTurnTo(-90);
    bothMove(0);
    brakehold();
    Delay(50);
    chain.move(127);
    intake.move(127);
    bothMove(600);
    Delay(400);
    bothMove(0);
    brakehold();
    Delay(150);

    // touch
    PidTurnTo(110);
    bothMove(0);
    brakehold();
    Delay(50);
    bothMove(600);
    Delay(400);
    arm.move(127);
    Delay(200);
    bothMove(0);
    
    arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);

}

void soloWPBlue() {
    // release
    chain.move(-127);

    //ring side

    //get stake    
    bothMove(-600);
    Delay(120);
    chain.move(0);
    Delay(350);
    bothMove(0);
    brakehold();
    Delay(30);
    for (int i = 0; i<3;i++){
        right.move_velocity(0);
        left.move_velocity(-400);
        Delay(120);
        left.move_velocity(0);
        right.move_velocity(-400);
        Delay(120);
    }
    bothMove(0);
    clamp.set_value(1);
    Delay(30);

    intake.move(127);
    chain.move(127);
    
    // get single ring
    PidTurnTo(270);
    bothMove(0);
    Delay(40);
    bothMove(600);
    Delay(200);
    bothMove(400);
    Delay(300);

    // go to other side
    PidTurnTo(140);
    bothMove(0);
    Delay(40);
    bothMove(600);
    Delay(520); //decreased from 650 to 540 backing before turn
    PidTurnTo(90);
    brakehold();
    bothMove(0);
    Delay(50);
    intake.move(127);
    bothMove(600);
    Delay(400); 
    bothMove(300);
    Delay(350); 
    chain.move(0);
    intake.move(0);
    PidTurnTo(130);
    intake.move(-600);
    Delay(600);
    PidTurnTo(90);
    chain.move(600);
    intake.move(600);
    delay(100);
    bothMove(600);
    Delay(400);
    //Change from 300 to 350
    // Part 2 starts

    // get stake
    chain.move(0);
    clamp.set_value(0);
    // Part 2 startes
    bothMove(0);
    Delay(150);
    bothMove(0);
    PidTurnTo(210);
    bothMove(-600);
    Delay(400);
    bothMove(0);
    brakehold();
    Delay(50);

    for (int i = 0; i<3;i++){
        right.move_velocity(0);
        left.move_velocity(-400);
        Delay(120);
        left.move_velocity(0);
        right.move_velocity(-400);
        Delay(120);
    }
    bothMove(0);
    clamp.set_value(1);
    Delay(20);

    // get ring
    PidTurnTo(90);
    bothMove(0);
    brakehold();
    Delay(50);
    chain.move(127);
    intake.move(127);
    bothMove(600);
    Delay(400);
    bothMove(0);
    brakehold();
    Delay(150);

    // touch
    PidTurnTo(290);
    bothMove(0);
    brakehold();
    Delay(50);
    bothMove(600);
    Delay(400);
    arm.move(127);
    Delay(200);
    bothMove(0);
    
    arm.set_brake_mode(E_MOTOR_BRAKE_HOLD);

}

void ringRushBlue(){
    bool armIntake = true;
    brakehold(); //Move to wall stake, facing rings
    chain.move(-600);
    bothMove(600);
    Delay(410);
    chain.move(0);
    Delay(200);
    bothMove(-90);
    arm.move(0);
    Delay(20);
    bothMove(0);
    PidTurnRightTo(-85);
    intake.move(600);
    chain.move(600);

    bothMove(200); //intake rings
    Delay(1000);
    bothMove(0);
    Delay(150);
    
    PidTurnTo(55,armIntake); //get mogo
    chain.move(0);
    Delay(80);
    bothMove(-300);
    Delay(600);
    for (int i = 0; i<2;i++){
        right.move_velocity(0);
        left.move_velocity(-400);
        Delay(120);
        left.move_velocity(0);
        right.move_velocity(-400);
        Delay(120);
    }
    arm.move(-127);
    bothMove(0);
    clamp.set_value(1);
    Delay(100);
    arm.move(0);
    PidTurnTo(100); //Get Center Ring
    chain.move(600);
    Delay(100);
    bothMove(600);
    Delay(300);
    Delay(160);
    bothMove(-100);
    Delay(40);
    chain.move(0);
    PidTurnTo(-160);
    chain.move(600);
    bothMove(400);
    Delay(700);
    bothMove(0);
    PidTurnTo(-78);
    chain.move(600);
    bothMove(600);
    Delay(800);
    intake.move(0);
    bothMove(-100);
    Delay(40);
    chain.move(0);
    PidTurnTo(-160);
    intake.move(-600);
    Delay(600);
    chain.move(600);
    PidTurnTo(-78);
    intake.move(600);
    bothMove(600);
    Delay(500);
    bothMove(-600);
    Delay(500);
    PidTurnTo(0);
    bothMove(600);
    Delay(600);
    arm.move(600);
    bothMove(0);
    

}
