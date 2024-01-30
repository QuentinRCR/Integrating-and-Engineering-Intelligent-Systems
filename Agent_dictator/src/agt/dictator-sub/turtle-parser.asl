// transform the rdf in believes explointable by the turtlebot
+!load_rdf 
   <- get("maqit.ttl") ; // load the rdf believes in the agent mind

      ?rdf(Config, "http://www.w3.org/1999/02/22-rdf-syntax-ns#type", "http://example.org/Config") ;
      ?rdf(Config, "http://example.org/traffic", TrafficConfig) ;
      ?rdf(TrafficConfig, "http://example.org/redLight", RedLightSign) ;
      +red_traffic_sign(RedLightSign) ;
      ?rdf(TrafficConfig, "http://example.org/greenLight", GreenLightSign) ;
      +green_traffic_sign(GreenLightSign) ;
      ?rdf(TrafficConfig, "http://example.org/distanceThreshold", Threshold) ;
      +traffic_distance_threshold(Threshold) ;
      .findall(WS, rdf(Config, "http://example.org/workstations", WS), WorkstationList) ;
      
      // assign the different stations to a sign
      for (.member(Workstation, WorkstationList)) {
         ?rdf(Workstation, "http://www.w3.org/1999/02/22-rdf-syntax-ns#type", WorkstationType) ;
         if (WorkstationType == "http://example.org/Workbench") {
            ?rdf(Workstation, "http://example.org/sign", WorkbenchSign) ;
            +workbench(WorkbenchSign) ;
         } elif (WorkstationType == "http://example.org/Packaging") {
            ?rdf(Workstation, "http://example.org/sign", PackagingSign) ;
            +packaging(PackagingSign) ;
         } elif (WorkstationType == "http://example.org/ItemStorage") {
            ?rdf(Workstation, "http://example.org/sign", ItemStorageSign) ;
            ?rdf(Workstation, "http://example.org/material", Material) ;
            +item_storage(Material, ItemStorageSign) ;
         }
      } ;

      // add receipes as believes
      .findall(R, rdf(R, "http://www.w3.org/1999/02/22-rdf-syntax-ns#type", "http://example.org/Recipe"), RecipeList) ;
      for (.member(Recipe, RecipeList)) {
         ?rdf(Recipe, "http://example.org/result", Result) ;
         .findall(I, rdf(Recipe, "http://example.org/ingredients", I), IngredientList) ;
         .map.create(Ingredients) ;
         for (.member(Ingredient, IngredientList)) {
            ?rdf(Ingredient, "http://example.org/material", Material) ;
            ?rdf(Ingredient, "http://example.org/amount", Amount) ;
            .map.put(Ingredients, Material, Amount) ;
         }
         +recipe(Result, Ingredients) ;
      } ;
      
      // Add every adresses of the topics 
      ?rdf(RosConfig, "http://www.w3.org/1999/02/22-rdf-syntax-ns#type", "http://example.org/ROSConfig") ;
      ?rdf(RosConfig, "http://example.org/seedUri", SeedUri) ;
      ?rdf(RosConfig, "http://example.org/seedForm", SeedForm) ;
      .findall(T, rdf(RosConfig, "http://example.org/topics", T), TopicList) ;
      for (.member(Topic, TopicList)) {
         ?rdf(Topic, "http://schema.org/name", Name) ;
         ?rdf(Topic, "http://schema.org/entryPoint", EntryPoint) ;
         ?rdf(Topic, "https://github.com/RobotWebTools/rosbridge_suite/blob/ros1/ROSBRIDGE_PROTOCOL.md#messageType", MessageType) ;
         .concat(SeedUri, EntryPoint, Uri) ;
         if (Name == "Move Action Goal") {
            +movement_action_uri(Uri) ;
            +movement_action_form([kv(SeedForm, MessageType)]) ;
         } elif (Name == "SLAM Action") {
            +slam_action_uri(Uri) ;
            +slam_action_form([kv(SeedForm, MessageType)]) ;
         } elif (Name == "SLAM Action Goal") {
            +slam_goal_uri(Uri) ;
            +slam_goal_form([kv(SeedForm, MessageType)]) ;
         } elif (Name == "SLAM Action Result") {
            +slam_result_uri(Uri) ;
            +slam_result_form([kv(SeedForm, MessageType)]) ;
         } elif (Name == "Audio Generation") {
            +speech_uri(Uri) ;
            +speech_form([kv(SeedForm, MessageType)]) ;
         } elif (Name == "Arm Control") {
            +arm_uri(Uri) ;
            +arm_form([kv(SeedForm, MessageType)]) ;
         } elif (Name == "Command Detection") {
            +commands_uri(Uri) ;
            +commands_form([kv(SeedForm, MessageType)]) ;
         } elif (Name == "Obstacle Detection") {
            +obstacles_uri(Uri) ;
            +obstacles_form([kv(SeedForm, MessageType)]) ;
         } elif (Name == "Sign Detection") {
            +signs_uri(Uri) ;
            +signs_form([kv(SeedForm, MessageType)]) ;
         } elif (Name == "Position Detection") {
            +map_position_uri(Uri) ;
            +map_position_form([kv(SeedForm, MessageType)]) ;
         }
      }
      .
