{ include("$jacamoJar/templates/common-cartago.asl") }
{ include("$jacamoJar/templates/common-moise.asl") }
{ include("./dictator-sub/turtle-parser.asl") } 

sign_position_margin(0.1) . //diffence of detected position to move a sign in the memory
destination_margin(0.25) . //if the robot is at this distance of its destination, the destination is considered as reached
update_signs(false) . // believe to try to update the new position of detected signs
traffic_stop(false) . // whether or not the robot need to stop because of a traffic light 
not_using_slam(true) . // whether or not the robot is in slam mode
holding("null") . // believe containing what the robot is currently holding

// Initialization
+!start
   <- .println("Subscribing to required topics") ;
      !load_rdf ; // create believes of the adresses from the rdf file using a parseur

      ?obstacles_uri(ObstaclesUri) ;
      ?obstacles_form(ObstaclesForm) ;
      watch(ObstaclesUri, ObstaclesForm) ; // subscribe to /obstacle

      ?commands_uri(CommandsUri) ;
      ?commands_form(CommandsForm) ;
      watch(CommandsUri, CommandsForm) ; // subscribe to audio_command
      
      ?signs_uri(SignsUri) ;
      ?signs_form(SignsForm) ;
      watch(SignsUri, SignsForm) ; // subscribe to /sign

      ?map_position_uri(MapUri) ;
      ?map_position_form(MapForm) ;
      watch(MapUri, MapForm) ; // subscribe to /amcl_pose

      ?slam_goal_uri(SlamGoalUri) ;
      ?slam_goal_form(SlamGoalForm) ;
      watch(SlamGoalUri, SlamGoalForm) ; // subscribe to /move_base_msgs/MoveBaseActionGoal

      ?slam_result_uri(SlamResultUri) ;
      ?slam_result_form(SlamResultForm) ;
      watch(SlamResultUri, SlamResultForm) ; // subscribe to /move_base_msgs/MoveBaseActionResult

      .

//===== HANDLE RECEIVED MESSAGES =====

// Handle audio commands
+json(Json)[source(S)] 
   :  commands_uri(S) &
      .member(kv(command, Command), Json) &
      .member(kv(args, Args), Json) &
      .member(kv(value, Value), Args) &
      .member(kv(type, Type), Args)
   <- !handle_command(Command, Type, Value) .

// Handle sign data
+json(Json)[source(S)]
   :  signs_uri(S) &
      .member(kv(angle, Angle), Json) &
      .member(kv(distance, Distance), Json) &
      .member(kv(sign, SignType), Json)
   <- !handle_signs(SignType, Distance, Angle * 3.1415926535/180) .

// Handle map localization
+json(Json)[source(S)]
   :  map_position_uri(S) &
      .member(kv(pose, Pose1), Json) &
      .member(kv(pose, Pose2), Pose1) &
      .member(kv(position, Position), Pose2) &
      .member(kv(orientation, Orientation), Pose2) &
      .member(kv(x, X), Position) &
      .member(kv(y, Y), Position) &
      .member(kv(w, W), Orientation) &
      .member(kv(z, Z), Orientation)
   <- !handle_position(X, Y, Z, W) .

// Handle obstacles
+json(Json)[source(S)]
   :  obstacles_uri(S) &
      .member(kv(data, Command), Json)
   <- !handle_obstacle(Command) .

// Intercept SLAM action server goal messages
+json(_)[source(S)]
   :  slam_goal_uri(S)
   <- -+not_using_slam(false) . // when a goal is sent, the robot pass in slam mode

// Intercept SLAM action server result messages
+json(_)[source(S)]
   :  slam_result_uri(S)
   <- -+not_using_slam(true) . // when a goal is sent, the robot leaves slam mode


//===== ACTION DICTATION =====

// Handle the position of the robot on the map
+!handle_position(X, Y, Z, W)
   <- lookupArtifact("orc", ArtifactId) ;
      focus(ArtifactId) ;
      convert(Z, W)[artifact_id(ArtifactId)] ; // convert the orientation in cartesian coordinates
      ?converted_value(Angle)[artifact_id(ArtifactId)] ; //retreive the value from the artifact
      
      -current_position(_, _, _) ; // update the current position
      +current_position(X, Y, Angle) .
-!handle_position(X, Y, Z, W)
   <- .print("Failed handling position data") .

// handle commands sent by the language processing program
+!handle_command(Command, Type, Value) 
   <- .print("Received command: ", Command, ", ", Type, ", ", Value);

      if (Command == "move") {
         !post_action(Type, Value) ; // directly transfer the movement command to the action server 
      } elif (Command == "toggle_sign") { // toggle the update of the sign position
         .print("TOGGLED") ;
         if (Type == "on") {
            .print("ON") ;
            -+update_signs(true) ;
         } elif (Type == "off") {
            .print("OFF") ;
            -+update_signs(false) ;
         } else {
            .print("Unknown command type for ", Command, ": ", Type) ;
         }
      } elif (Command == "action") {
         if (Type == "take") {
            !take(math.ceil(Value)) ; // pick up a determined item
         } elif (Type == "drop") {
            !drop ; //drop the item in the arm
         } elif (Type == "make") {
            !make(math.ceil(Value)) ; //make an object of id Value
         } else {
            .print("Unknown command type for ", Command, ": ", Type) ;
         }
      } else {
         .print("Unknown command: ", Command) ;
      }.
-!handle_command(Command, Type, Value) 
   <- .print("Failed handling command") .


+!handle_signs(SignType, Distance, Angle)
   <- if (red_traffic_sign(Red) & SignType == Red) { //check we have a red traffic light
         if (traffic_distance_threshold(Threshold) & Distance <= Threshold) { //make sur the traffic light is close enougt
            -+traffic_stop(true) ;
            .print("Detected RED traffic light, stopping...");
            !post_action("forward", 0.00001) ; // force the robot to stop if it was mooving  
         }
      } elif (green_traffic_sign(Green) & SignType == Green) { //check we have a green traffic light
         if (traffic_distance_threshold(Threshold) & Distance <= Threshold) { //make sur the traffic light is close enougt
            -+traffic_stop(false) ; //remove the stop believe
            .print("Detecting GREEN traffic light, movement allowed");
         }
      } elif (update_signs(Status) & Status == true) { //Update the position on the signs based on the position detected by the camera
         ?current_position(CurrentX, CurrentY, CurrentYaw) ;

         // calculate the position of the sign is the coordinates of the map
         +calculated_sign_position(CurrentX + Distance * math.cos(Angle + CurrentYaw), CurrentY + Distance * math.sin(Angle + CurrentYaw)) ;
         ?calculated_sign_position(NewX, NewY) ;
         -calculated_sign_position(NewX, NewY) ;

         ?sign_position(SignType, OldX, OldY) ; //get the old sign position
         ?sign_position_margin(Radius) ;

         //check the sign really is in a very different position than previously thought
         if (math.sqrt((NewX - OldX) * (NewX - OldX) + (NewY - OldY) * (NewY - OldY)) > Radius) {
            -sign_position(SignType, OldX, OldY) ;
            +sign_position(SignType, NewX, NewY) ; // update the sign position
            .print("Updated the position of ", SignType);
         } ;
      } .
-!handle_signs(SignType, Distance, Angle)
   <- .print("Failed handling sign data") .
   
+!handle_obstacle(Command)
   : not_using_slam(true) // disable the obstacle detection when the robot is in slam mode, to avoid intereferences 
   <- if (Command == "left") {
         !post_action("right", 90) ; // obstacle on the left => turn right 
      } elif (Command == "right") {
         !post_action("left", 90) ; // obstacle on the right => turn left
      } elif (Command == "front") {
         !post_action("backward", 0.1) ; // obstacle in front => go backward
      } elif (Command == "back") {
         !post_action("forward", 0.2) ; // obstacle in the back => go forward
      } .
-!handle_obstacle(Command) // if the plan was not possible because of a red trafic line
   :  not_using_slam(false)
   <- .print("SLAM: could not handle obstacle") .
-!handle_obstacle(Command)
   <- .print("Failed handling obstacle") .

// ask the robot to move
+!post_action(Direction, Value)
   :  movement_action_uri(URI) & movement_action_form(Form) & traffic_stop(false)
   <- post(URI, json([kv(direction, Direction), kv(distance, Value)]), Form) . // post to the action server
-!post_action(Direction, Value)
   :  traffic_stop(true)  // if the movement was prevented by a trafic light
   <- .print("Red light - cannot move").

// simulate a command to the arm
+!take(Material)
   <- .println("Picking up item ", Material) ;
      -+holding(Material) .

// simulate a command to the arm
+!drop
   <- ?holding(Material) ;
      .println("Dropping  off item ", Material) ;
      -+holding("null") .

// reach a material using slam
+!reach(Material)
   <- if (Material == 0) {
         .println("Going to workbench") ;
         ?workbench(Sign) ; //get the sign corresponding to the workbench
      } elif (Material == -1) {
         .println("Going to packaging station") ;
         ?packaging(Sign) ;
      } else {
         .println("Reaching for item ", Material) ;
         ?item_storage(Material, Sign) ;
      } ;
      ?sign_position(Sign, SignX, SignY) ; // get the position of the sign
      !goto(SignX, SignY) ; //go to the position of the sign

      // While the robot did not reach the sign, wait
      while (current_position(X, Y, _) & destination_margin(Margin) & math.sqrt((X - SignX) * (X - SignX) + (Y - SignY) * (Y - SignY)) > Margin) {
         // if (not_using_slam(Status) & Status == true) {
         //    .println("SLAM interrupted before reaching destination: repeating command") ;
         //    !goto(SignX, SignY) ;
         // } ;
         .wait(2000) ;
      } ;
      // !goto(X, Y) ; // force the robot to stop
      .

// perform a list of tasks to make a defined object
+!make(Result)
   <- .println("Making a ", Result) ;
      ?recipe(Result, Ingredients) ; // get the recipe

      // for every material, pick it up, and bring it to the workbench
      for (.map.key(Ingredients, Material) & .map.get(Ingredients, Material, Amount)) {
         for (.range(I, 1, Amount)) {
            !reach(Material) ;
            !take(Material) ;
            !reach(0) ; // Go to workbench
            !drop ;
         }
      } ;
      // Instantly craft because the robot is already at the workbench
      .println("Crafted item ", Result, " with success!") ;

      !take(Result) ; // Pick up the crafted item

      !reach(-1) ; // Go to packaging station

      !drop ;
      .println("Dropped off item ", Result, " for packaging!") ;
      .

// got to said coordinates using the slam action server
+!goto(X, Y)
   :  slam_action_uri(URI) &
      slam_goal_form(Form)
   <- -+not_using_slam(false);
      post(URI, json([
         kv(target_pose, [
            kv(header, [
               kv(seq, 0), 
               kv(stamp, [
                  kv(secs, 1705396052),
                  kv(nsecs, 722924975)
               ]),
               kv(frame_id, "map")
            ]),
            kv(pose, [
               kv(orientation, [
                  kv(x, 0),
                  kv(z, 0.8529481786779419), // always use the same orientation
                  kv(w, 0.5219955981519209),
                  kv(y, 0)
               ]),
               kv(position, [
                  kv(x, X),
                  kv(z, 0),
                  kv(y, Y)
               ])
            ])
         ])
      ]), Form) .