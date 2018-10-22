// -------------------------------------------------------------- -*- C++ -*-
// File: blend.cpp
// Version 12.7.0  
// --------------------------------------------------------------------------
// Licensed Materials - Property of IBM
// 5725-A06 5725-A29 5724-Y48 5724-Y49 5724-Y54 5724-Y55 5655-Y21
// Copyright IBM Corporation 2000, 2016. All Rights Reserved.
//
// US Government Users Restricted Rights - Use, duplication or
// disclosure restricted by GSA ADP Schedule Contract with
// IBM Corp.
// --------------------------------------------------------------------------
//
// blend.cpp -- A blending problem

/* ------------------------------------------------------------

Problem Description
-------------------

Goal is to blend four sources to produce an alloy: pure metal, raw
materials, scrap, and ingots.

Each source has a cost.
Each source is made up of elements in different proportions.
Alloy has minimum and maximum proportion of each element.

Minimize cost of producing a requested quantity of alloy.

------------------------------------------------------------ */


#include <ilcplex/ilocplex.h>
#include <json.hpp>
#include <vector>
using json = nlohmann::json;


ILOSTLBEGIN

IloInt nbElements, nbRaw, nbScrap, nbIngot;
IloNum alloy;
IloNumArray nm, nr, ns, ni, p, P;

IloNumArray2 PRaw, PScrap, PIngot;


void define_data(IloEnv env) {
   nbElements = 3;
   nbRaw      = 2;
   nbScrap    = 2;
   nbIngot    = 1;
   alloy      = 71;
   nm = IloNumArray(env, nbElements, 22.0, 10.0, 13.0);
   nr = IloNumArray(env, nbRaw, 6.0, 5.0);
   ns = IloNumArray(env, nbScrap, 7.0, 8.0);
   ni = IloNumArray(env, nbIngot, 9.0);
   p = IloNumArray(env, nbElements, 0.05, 0.30, 0.60);
   P = IloNumArray(env, nbElements, 0.10, 0.40, 0.80);
   PRaw   = IloNumArray2(env, nbElements);
   PScrap = IloNumArray2(env, nbElements);
   PIngot = IloNumArray2(env, nbElements);
   PRaw[0] = IloNumArray(env, nbRaw, 0.20, 0.01);
   PRaw[1] = IloNumArray(env, nbRaw, 0.05, 0.00);
   PRaw[2] = IloNumArray(env, nbRaw, 0.05, 0.30);

   PScrap[0] = IloNumArray(env, nbScrap, 0.00, 0.01);
   PScrap[1] = IloNumArray(env, nbScrap, 0.60, 0.00);
   PScrap[2] = IloNumArray(env, nbScrap, 0.40, 0.70);

   PIngot[0] = IloNumArray(env, nbIngot, 0.10);
   PIngot[1] = IloNumArray(env, nbIngot, 0.45);
   PIngot[2] = IloNumArray(env, nbIngot, 0.45);
}


int
main(int, char**)
{
   /*
	json j;
	j["pi"] = 3.141;

	// add a Boolean that is stored as bool
	j["happy"] = true;

	// add a string that is stored as std::string
	j["name"] = "Niels";

	// add another null object by passing nullptr
	j["nothing"] = nullptr;

	// add an object inside the object
	j["answer"]["everything"] = 42;

	// add an array that is stored as std::vector (using an initializer list)
	j["list"] = { 1, 0, 2 };

	// add another object (using an initializer list of pairs)
	j["object"] = { { "currency", "USD" },{ "value", 42.99 } };

	// instead, you could also write (which looks very similar to the JSON above)
	json j2 = {
		{ "pi", 3.141 },
		{ "happy", true },
		{ "name", "Niels" },
		{ "nothing", nullptr },
		{ "answer",{
			{ "everything", 42 }
		} },
		{ "list",{ 1, 0, 2 } },
		{ "object",{
			{ "currency", "USD" },
			{ "value", 42.99 }
		} }
	};

	std::cout << j.dump(4) << std::endl;
	*/

	IloEnv env;
   try {
      IloInt j;
   
      define_data(env);
   
      IloModel model(env);
   
      IloNumVarArray m(env, nbElements, 0.0, IloInfinity);
      IloNumVarArray r(env, nbRaw,   0.0, IloInfinity);
      IloNumVarArray s(env, nbScrap, 0.0, IloInfinity);
      IloNumVarArray i(env, nbIngot, 0.0, 100000);
      IloNumVarArray e(env, nbElements);
   
      // Objective Function: Minimize Cost
      model.add(IloMinimize(env, IloScalProd(nm, m) + IloScalProd(nr, r) +
                                 IloScalProd(ns, s) + IloScalProd(ni, i)  ));
   
      // Min and max quantity of each element in alloy
      for (j = 0; j < nbElements; j++) {
         e[j] = IloNumVar(env, p[j] * alloy, P[j] * alloy);
      }
   
      // Constraint: produce requested quantity of alloy
      model.add(IloSum(e) == alloy);
   
      // Constraints: Satisfy element quantity requirements for alloy
      for (j = 0; j < nbElements; j++) {
         model.add(e[j] == m[j] + IloScalProd(PRaw[j], r)
                                + IloScalProd(PScrap[j], s)
                                + IloScalProd(PIngot[j], i));
      }
   
      // Optimize
      IloCplex cplex(model);
      cplex.setOut(env.getNullStream());
      cplex.setWarning(env.getNullStream());
      cplex.solve();
   
      if (cplex.getStatus() == IloAlgorithm::Infeasible)
         env.out() << "No Solution" << endl;

      env.out() << "Solution status: " << cplex.getStatus() << endl;
   
      // Print results
      env.out() << "Cost:" << cplex.getObjValue() << endl;
      env.out() << "Pure metal:" << endl;
      for(j = 0; j < nbElements; j++)
         env.out() << j << ") " << cplex.getValue(m[j]) << endl;
      env.out() << "Raw material:" << endl;
      for(j = 0; j < nbRaw; j++)
         env.out() << j << ") " << cplex.getValue(r[j]) << endl;
      env.out() << "Scrap:" << endl;
      for(j = 0; j < nbScrap; j++)
         env.out() << j << ") " << cplex.getValue(s[j]) << endl;
      env.out() << "Ingots : " << endl;
      for(j = 0; j < nbIngot; j++)
         env.out() << j << ") " << cplex.getValue(i[j]) << endl;
      env.out() << "Elements:" << endl;
      for(j = 0; j < nbElements; j++)
         env.out() << j << ") " << cplex.getValue(e[j]) << endl;
	  
	  IloNum newM[1000];
	  IloNum newR[1000];
	  IloNum newS[1000];
	  IloNum newI[1000];
	  IloNum newE[1000];

	  for (j = 0; j < nbElements; j++) {
		  newM[j] = (cplex.getValue(m[j]));
	  }
	  for (j = 0; j < nbRaw; j++) {
		  newR[j] = (cplex.getValue(r[j]));
	  }
	  for (j = 0; j < nbScrap; j++) {
		  newS[j] = (cplex.getValue(s[j]));
	  }
	  for (j = 0; j < nbIngot; j++) {
		  newI[j] = (cplex.getValue(i[j]));
	  }
	  for (j = 0; j < nbElements; j++) {
		  newE[j] = (cplex.getValue(e[j]));
	  }

	  /*
	  json output = {
		  {"Solution status", cplex.getStatus()}
		  { "Cost", cplex.getObjValue()},
		  {"Pure metal", },
	  { "Raw material", for (j = 0; j < nbElements; j++) {
		  cplex.getValue(r[j]);}},
		  { "Scrap", for (j = 0; j < nbElements; j++) {
			  cplex.getValue(s[j]);}},
			  { "Ingots", for (j = 0; j < nbElements; j++) {
					  cplex.getValue(i[j]);}},
					  { "Elements", for (j = 0; j < nbElements; j++) {
								  cplex.getValue(e[j]);
} },
	  };
	  */
	  

   }
   catch (IloException& ex) {
      cerr << "Error: " << ex << endl;
   }
   catch (...) {
      cerr << "Error" << endl;
   }
   env.end();



   system("pause");
   return 0;
}

/*
Cost:653.554
Pure metal:
0) 0
1) 0
2) 0
Raw material:
0) 0
1) 0
Scrap:
0) 17.059
1) 30.2311
Ingots : 
0) 32.4769
Elements:
0) 3.55
1) 24.85
2) 42.6
*/
