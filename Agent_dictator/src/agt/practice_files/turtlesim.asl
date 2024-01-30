{ include("$jacamoJar/templates/common-cartago.asl") }
{ include("$jacamoJar/templates/common-moise.asl") }

twist_uri(ResultURI) :- seed_uri(SeedURI) & .concat(SeedURI,"/cmd_vel",ResultURI) .
twist_form([kv(SeedForm, "geometry_msgs/Twist")]) :- seed_form(SeedForm) .
twist_payload(json([ kv(linear, [kv(x,0), kv(y,1),kv(z,0)]), kv(angular, [kv(x,0), kv(y,0),kv(z,0)]) ])) . 


+!start 
   :  twist_uri(TwistURI) & twist_form(TwistForm) & twist_payload(TwistPayload)
   <- .println("Starting") ;
      put(TwistURI, TwistPayload, TwistForm) .

+json(Msg) <- .print("observed ", Msg) .

