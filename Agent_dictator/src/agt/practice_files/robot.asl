{ include("$jacamoJar/templates/common-cartago.asl") }
{ include("$jacamoJar/templates/common-moise.asl") }


action_server_uri(URI) :- seed_uri(SeedURI) & .concat(SeedURI,"/move_action", URI) .
action_msg_form([kv(SeedForm, "/move_action/Turtlebot_moveActionGoal")]) :- seed_form(SeedForm) .
payload(json([ kv(direction, "left"), kv(distance, 90) ])) . 
payload_f(json([ kv(direction, "forward"), kv(distance, 0.2) ])) . 

+!start 
   :  action_server_uri(URII) &
      .print(URII) &
      action_msg_form(Form) &
      .print(Form) &
      payload(Payload) &
      .print(Payload)
      <- .println("Starting post 1") ;
      !post1 .

+!post1
   :  action_server_uri(URI) & action_msg_form(Form) & payload(Payload)
   <- .print(URI);
      .print(Form);
      .print(Payload); 
      post(URI, Payload, Form) .

// +!post2
//    :  action_server_uri(URI) & action_msg_form(Form) & payload_f(Payload)
//    <- .wait(100) ;
//       .print("2"); 

//       post(URI, Payload, Form) .

+json(Msg) <- .print("observed ", Msg) .