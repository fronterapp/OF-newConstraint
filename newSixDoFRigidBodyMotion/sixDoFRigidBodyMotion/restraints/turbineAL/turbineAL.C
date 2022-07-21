/*---------------------------------------------------------------------------*\
  =========                 |
  \\      /  F ield         | OpenFOAM: The Open Source CFD Toolbox
   \\    /   O peration     |
    \\  /    A nd           | www.openfoam.com
     \\/     M anipulation  |
-------------------------------------------------------------------------------
    Copyright (C) 2011-2016 OpenFOAM Foundation
-------------------------------------------------------------------------------
License
    This file is part of OpenFOAM.

    OpenFOAM is free software: you can redistribute it and/or modify it
    under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenFOAM is distributed in the hope that it will be useful, but WITHOUT
    ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
    FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
    for more details.

    You should have received a copy of the GNU General Public License
    along with OpenFOAM.  If not, see <http://www.gnu.org/licenses/>.

\*---------------------------------------------------------------------------*/

#include "turbineAL.H"
#include "addToRunTimeSelectionTable.H"
#include "sixDoFRigidBodyMotion.H"
#include "fvMesh.H"

// * * * * * * * * * * * * * * Static Data Members * * * * * * * * * * * * * //

namespace Foam
{
namespace sixDoFRigidBodyMotionRestraints
{
    defineTypeNameAndDebug(turbineAL, 0);

    addToRunTimeSelectionTable
    (
        sixDoFRigidBodyMotionRestraint,
        turbineAL,
        dictionary
    );
}
}


// * * * * * * * * * * * * * * * * Constructors  * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::turbineAL::turbineAL
(
    const word& name,
    const dictionary& sDoFRBMRDict
)
:
    sixDoFRigidBodyMotionRestraint(name, sDoFRBMRDict)
{   
    read(sDoFRBMRDict);
    createTurbineDict();
}


// * * * * * * * * * * * * * * * * Destructor  * * * * * * * * * * * * * * * //

Foam::sixDoFRigidBodyMotionRestraints::turbineAL::~turbineAL()
{}


// * * * * * * * * * * * * * * Member Functions  * * * * * * * * * * * * * * //

void Foam::sixDoFRigidBodyMotionRestraints::turbineAL::restrain
(
    const sixDoFRigidBodyMotion& motion,
    vector& restraintPosition,
    vector& restraintForce,
    vector& restraintMoment
) const
{
    dictionary loadDict = dictionary();

    // If turbine loads dictionary exists
    if(time_.foundObject<IOdictionary>("turbineSixDoFLoads"))
    {
        //- Read turbine load dict from the 'turbineSixDoFLoads' IOdictionary
        loadDict = time_.lookupObject<IOdictionary>("turbineSixDoFLoads");
    }

    // Get load data from dictionary
    loadDict.lookup("force") >> restraintForce;
    loadDict.lookup("moment") >> restraintMoment;
    loadDict.lookup("refPoint") >> restraintPosition;

    if (motion.report())
    {
        Info<< " turbine force application point " << restraintPosition
            << " turbine force vector " << restraintForce
            << " turbine moment vector " << restraintMoment
            << endl;
    }
}


bool Foam::sixDoFRigidBodyMotionRestraints::turbineAL::read
(
    const dictionary& sDoFRBMRDict
)
{
    sixDoFRigidBodyMotionRestraint::read(sDoFRBMRDict);
    return true;
}


void Foam::sixDoFRigidBodyMotionRestraints::turbineAL::write
(
    Ostream& os
) const
{

}

void Foam::sixDoFRigidBodyMotionRestraints::turbineAL::createTurbineDict()
{
    // Create dictionary if it has not been created before
    if(!time_.foundObject<IOdictionary>("turbineSixDoFLoads"))
    {
        dictionary motionDict;
        time_.store
        (   
            new IOdictionary
            (   
                IOobject
                (   
                    "turbineSixDoFLoads",
                    time_.timeName(),
                    time_,
                    IOobject::NO_READ,
                    IOobject::AUTO_WRITE
                ),
                motionDict
            )
        );

        Info << "Rigid body IOdictionary 'turbineSixDoFLoads' created" << endl;

        initialiseTurbineDict();
    }
}

void Foam::sixDoFRigidBodyMotionRestraints::turbineAL::initialiseTurbineDict()
{
    if(time_.foundObject<IOdictionary>("turbineSixDoFLoads"))
    {
        // Open and write
        const dictionary& loadsDict = 
            time_.lookupObject<IOdictionary>("turbineSixDoFLoads");

        // Initialise the load data with zero entries
        dictionary updateDbDictionary = &loadsDict;
        updateDbDictionary.set("force", vector::zero);
        updateDbDictionary.set("moment", vector::zero);
        updateDbDictionary.set("refPoint", vector::zero);

        // Needed to update the IOdictionary
        const_cast<dictionary& > (loadsDict) = updateDbDictionary;
        
        Info << "Initialising 'turbineSixDoFLoads' IOdictionary" << endl;
    }
}

// ************************************************************************* //
