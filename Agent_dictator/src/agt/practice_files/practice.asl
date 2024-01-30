{ include("$jacamoJar/templates/common-cartago.asl") }
{ include("$jacamoJar/templates/common-moise.asl") }


// has_Sample(Individual1, Individual2) :- rdf(Individual1, "https://w3id.org/bot#hasSpace", Individual2) .
// hosts(Individual1, Individual2) :- rdf(Individual1, "http://www.w3.org/ns/sosa/hosts", Individual2) .
// type(Individual1, Individual2) :- rdf(Individual1, "http://www.w3.org/1999/02/22-rdf-syntax-ns#type", Individual2) .
// label(Individual1, Individual2) :- rdf(Individual1, "http://www.w3.org/2000/01/rdf-schema#label", Individual2) .
// see_also(Individual1, Individual2) :- rdf(Individual1, "http://www.w3.org/2000/01/rdf-schema#seeAlso", Individual2) .
// location(Individual1, Individual2) :- rdf(Individual1, "http://www.w3.org/2003/01/geo/wgs84_pos#location", Individual2) .

!start.

+!start : true
   <- .println("Starting") ;
      get("maqit.ttl") ;
		.

+json(Msg) <- .print("observed ", Msg) .