#include "../include/main.h"
#include "../include/functions.h"


std::shared_ptr<AsyncPositionController<double, double>> liftControl =
    AsyncPosControllerBuilder().withMotor({BRLift,BLLift}).build();
std::shared_ptr<AsyncPositionController<double, double>> fourbar =
    AsyncPosControllerBuilder().withMotor({FBRPort,FBLPort}).build();

//This file has all of the autonomous
void disabledAuton(){

}

void Robot_inspection(){
  std::shared_ptr<ChassisController> driveauton =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withGains(
  {0.002, 0, 0.0001}, // Distance controller gains
  {0.001, 0, 0.0001} // Turn controller gains
  )
  .withMaxVelocity(200)

  .withDerivativeFilters(
        std::make_unique<AverageFilter<3>>()
    )
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .withOdometry() // use the same scales as the chassis (above)
  .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)

  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController2 =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();
    driveauton->moveDistance(25_in);


}
void skills(){
  std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();

  // Target location path
  profileController->generatePath({
      {64_in, 0_in, 0_deg},
      {0_ft, 0_ft, 0_deg}},
      "first_move"
  );
  profileController->generatePath({
      {0_in, 0_in, 0_deg},
      {30_in, 0_in, 0_deg}},
      "retreat"
  );

  profileController->setTarget("first_move",true);
  delay(1500);
  bliftmove(100);
  delay(500);
  profileController->setTarget("retreat",true);
  delay(1000);
}


void Drive(){

  std::shared_ptr<ChassisController> driveauton =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withGains(
  {0.002, 0, 0.0001}, // Distance controller gains
  {0.001, 0, 0.0001} // Turn controller gains
  )
  .withMaxVelocity(200)

  .withDerivativeFilters(
        std::make_unique<AverageFilter<3>>()
    )
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .withOdometry() // use the same scales as the chassis (above)
  .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)

  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();
profileController->generatePath({
      {0_ft, 0_ft, 0_deg},
      {40_in, 0_ft, 0_deg}},
      "Gotonumogo"
    );
    profileController->generatePath({
          {0_ft, 0_ft, 0_deg},
          {40_in, 0_ft, 0_deg}},
          "return"
        );
//driveauton->moveDistance(40_in);
profileController->setTarget("Gotonumogo");
delay(1600);
bliftmove(100);
delay(100);
profileController->generatePath({
      {0_ft, 0_ft, 0_deg},
      {35_in, 0_ft, 0_deg}},
      "Gotoamogo"
    );
profileController->setTarget("Gotoamogo",true);
delay(800);
bliftmove(100);
delay(100);
delay(15000);
}

void AWP1(){

    std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)

    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

    std::shared_ptr<ChassisController> driveautonnotpid =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withMaxVelocity(200)

    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
    .build(); // build an odometry chassis

    std::shared_ptr<AsyncMotionProfileController> profileController2 =
    AsyncMotionProfileControllerBuilder()
      .withLimits({
        1.0, // Maximum linear velocity of the Chassis in m/s
        2.0, // Maximum linear acceleration of the Chassis in m/s/s
        10.0 // Maximum linear jerk of the Chassis in m/s/s/s
      })
      .withOutput(driveauton)
      .buildMotionProfileController();
driveauton->moveDistance(7_in);
    bliftmove(100);
    delay(100);
     delay(1000);
    bliftmove(100);
}
void AWP2(){

  std::shared_ptr<ChassisController> driveauton =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withGains(
  {0.002, 0, 0.0001}, // Distance controller gains
  {0.001, 0, 0.0001} // Turn controller gains
  )
  .withMaxVelocity(200)

  .withDerivativeFilters(
        std::make_unique<AverageFilter<3>>()
    )
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .withOdometry() // use the same scales as the chassis (above)
  .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)

  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController2 =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })

    .withOutput(driveauton)
    .buildMotionProfileController();

    driveauton->moveDistance(-28_in);
    bliftmove(100);
    driveauton->moveDistance(20_in);
    delay(300);
    bliftmove(100);


}

void TEST_GO_1() {
  std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();

  // Target location path
  profileController->generatePath({
      {0_in, 0_in, 0_deg},
      {64_ft, 0_ft, 0_deg}},
      "first_move"
  );
  profileController->generatePath({
      {0_in, 0_in, 0_deg},
      {60_in, 0_in, 0_deg}},
      "retreat"
  );
driveauton->moveDistance(-64_in);
delay(100);
bliftmove(100);
delay(500);
driveauton->moveDistance(60_in);
bliftmove(100);
delay(500);
}

void TEST_GO_2() {
  std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(200)
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();

  // Target location path
  profileController->generatePath({
      {0_in, 0_in, 0_deg},
      {52_ft, 0_ft, 0_deg}},
    "first_move"
  );
  profileController->generatePath({
    {0_ft, 0_ft, 0_deg},
    {40_in, 0_in, 0_deg}},
    "2ndnumogo"
  );
  profileController->generatePath({
    {0_ft, 0_ft, 0_deg},
    {50_in, 0_in, 0_deg}},
    "backout"
  );
  profileController->generatePath({
    {0_ft, 0_ft, 0_deg},
    {45_in, 0_in, 0_deg}},
    "idk"
  );

//profileController->setTarget("first_move",true);
//bliftmove.move_relative(7500, 10);

  driveauton->moveDistance(-52_in);
  bliftmove(100);
  delay(300);
  driveauton->moveDistance(40_in);
  driveauton->turnAngle(-35_deg);
  bliftmove(100);
  driveauton->turnAngle(72_deg);
  //could possibly also be 75 degrees
  driveauton->moveDistance(-50_in);
  bliftmove(100);
  delay(300);
  driveauton->moveDistance(45_in);
  bliftmove(100);
}

void TEST_GO_3() {
  std::shared_ptr<ChassisController> driveauton =
    ChassisControllerBuilder()
    .withMotors({FLPort,BLPort},{FRPort,BRPort})
    .withGains(
    {0.002, 0, 0.0001}, // Distance controller gains
    {0.001, 0, 0.0001} // Turn controller gains
    )
    .withMaxVelocity(200)
    .withDerivativeFilters(
          std::make_unique<AverageFilter<3>>()
      )
    // green gearset, 4 inch wheel diameter, 15 inch wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
    .withOdometry() // use the same scales as the chassis (above)
    .buildOdometry(); // build an odometry chassis

  std::shared_ptr<ChassisController> driveautonnotpid =
  ChassisControllerBuilder()
  .withMotors({FLPort,BLPort},{FRPort,BRPort})
  .withMaxVelocity(100)
  // green gearset, 4 inch wheel diameter, 15 inch wheel track
  .withDimensions(AbstractMotor::gearset::green, {{4_in, 16_in}, imev5GreenTPR})
  .build(); // build an odometry chassis

  std::shared_ptr<AsyncMotionProfileController> profileController =
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(driveauton)
    .buildMotionProfileController();

  // Target location path
  profileController->generatePath({
      {54_in, 0_in, 0_deg},
      {0_ft, 0_ft, 0_deg}},
      "first_move"
  );
  profileController->generatePath({
      {0_ft, 0_ft, 0_deg},
      {50_in, 0_in, 0_deg}},
      "run_home"
  );

  driveauton->moveDistance(-52_in);
  bliftmove(100);
  delay(300);
  driveauton->moveDistance(38_in);
  driveauton->turnAngle(-35_deg);
  bliftmove(100);
  driveauton->turnAngle(75_deg);
  driveauton->moveDistance(-47_in);
  bliftmove(100);
  delay(300);
  driveauton->moveDistance(40_in);
  bliftmove(100);
}
