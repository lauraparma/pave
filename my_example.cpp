//
// Laura - simulator of road pavements 
//
//

// A very simple example that can be used as template project for
// a Chrono::Engine simulator with 3D view.

#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/assets/ChTexture.h"
#include "chrono/assets/ChColorAsset.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/assets/ChPointPointDrawing.h"
#include "chrono/physics/ChLoadsBody.h"
#include "chrono/physics/ChLoadContainer.h"

// Use the namespace of Chrono

using namespace chrono;
using namespace chrono::irrlicht;

// Use the main namespaces of Irrlicht

using namespace irr;
using namespace irr::core;
using namespace irr::scene;
using namespace irr::video;
using namespace irr::io;
using namespace irr::gui;

int main(int argc, char* argv[]) {
    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);
    
    // Create a Chrono physical system
    ChSystem mphysicalSystem;

    // Create the Irrlicht visualization (open the Irrlicht device,
    // bind a simple user interface, etc. etc.)
    ChIrrApp application(&mphysicalSystem, L"A simulator for stone pavements", core::dimension2d<u32>(800, 600),
                         false);  // screen dimensions

    // Easy shortcuts to add camera, lights, logo and sky in Irrlicht scene:
    //application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 3, -4),
                                 core::vector3df(0, 0, 0));  // to change the position of camera
    // application.AddLightWithShadow(vector3df(1,25,-5), vector3df(0,0,0), 35, 0.2,35, 55, 512, video::SColorf(1,1,1));

    //======================================================================

    // HERE YOU CAN POPULATE THE PHYSICAL SYSTEM WITH BODIES AND LINKS.
    //
    // An example: a pendulum.

    // 1-Create a floor that is fixed (that is used also to represent the absolute reference)

    auto floorBody = std::make_shared<ChBodyEasyBox>(10, 2, 10,  // x, y, z dimensions
                                                     1000,       // density
                                                     false,      // no contact geometry
                                                     true        // enable visualization geometry
                                                     );
    ChVector<> mypos;
    mypos.Set(0, -1, 0);  // or:   mypos.y() = -2;
    floorBody->SetPos(mypos);
    floorBody->SetBodyFixed(true);

    mphysicalSystem.Add(floorBody);

    // contains all 'loads' (ie. bushings)
    auto my_loadcontainer = std::make_shared<ChLoadContainer>();
    mphysicalSystem.Add(my_loadcontainer);


    // Some parameters:
	// (da non cambiare qui perche' l'apparecchiatura a spina di pesce si incastra soltanto se
	// giunto_gap_x = giunto_gap_z, 2*size_tile_z + giunto_gap_x = size_tile_x)

    double size_tile_x = 0.3;
    double size_tile_y = 0.1;
    double size_tile_z = 0.145;
    double tile_density = 1500; 
    double tile_gap_y = 0.05; // respect to soil
    double giunto_gap_x = 0.01;
    double giunto_gap_z = 0.01;

    // Bushing soil-block
    //     vertical:
    double vert_stiffness = 12500;  // N/m
    double vert_damping   = 100;    // N/ m/s
    double vert_yeld      = 10000;  // N/m
    //     horizontal  (shear) (in two dim?)
    double hor_stiffness = 12500;  // N/m
    double hor_damping   = 100;    // N/ m/s
    double hor_yeld      = 100000;  // N/m

    // Bushing block-block
    //     orthogonal
    double ortho_stiffness_giunto = 10500;  // N/m      
    double ortho_damping_giunto   = 100;    // N/ m/s
    double ortho_yeld_giunto      = 100000;    // N/ m/s
    //     shear (in two dim?)
    double shear_stiffness_giunto = 3500;  // N/m
    double shear_damping_giunto   = 100;    // N/ m/s
    double shear_yeld_giunto      = 100000;  // N/ m/s



    std::shared_ptr<ChBodyEasyBox> piastrelle[10][8];

    for (int iz =  0; iz < 8; ++iz) {
        for (int ix =  0; ix < 10; ++ix) {

            // 2-Create a tile

            piastrelle[ix][iz] = std::make_shared<ChBodyEasyBox>(size_tile_x, size_tile_y, size_tile_z, 
                                                                tile_density,      // density
                                                                true,         // ok, contact geometry
                                                                true          // enable visualization geometry
                                                                );
            double offset_x = ix * (size_tile_x + giunto_gap_x);
            double offset_z = iz * (size_tile_z + giunto_gap_z);

            piastrelle[ix][iz]->SetPos(ChVector<>(
                                                    offset_x, 
                                                    size_tile_y*0.5 + tile_gap_y, 
                                                    offset_z
                                                )); 

            mphysicalSystem.Add(piastrelle[ix][iz]);

        }
    }

    for (int iz =  0; iz < 8; ++iz) {
        for (int ix =  0; ix < 10; ++ix) {

            double offset_x = ix * (size_tile_x + giunto_gap_x);
            double offset_z = iz * (size_tile_z + giunto_gap_z);

            /* old molle!
            // 3-Create the vertical springs: soil-tile

            auto molla1 = std::make_shared<ChLinkSpring>();
            molla1->Initialize(piastrelle[ix][iz], floorBody, false, ChVector<>(-size_tile_x*0.5 + offset_x, tile_gap_y, -size_tile_z*0.5 + offset_z), ChVector<>(-size_tile_x*0.5 + offset_x, 0, -size_tile_z*0.5 + offset_z), true);
            molla1->Set_SpringK(vert_stiffness);
            molla1->Set_SpringR(vert_damping);
            auto molla1_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
            molla1->AddAsset(molla1_vis);
            mphysicalSystem.Add(molla1);

            auto molla2 = std::make_shared<ChLinkSpring>(); 
            molla2->Initialize(piastrelle[ix][iz], floorBody, false, ChVector<>( size_tile_x*0.5 + offset_x, tile_gap_y, -size_tile_z*0.5 + offset_z), ChVector<>( size_tile_x*0.5 + offset_x, 0, -size_tile_z*0.5 + offset_z), true);
            molla2->Set_SpringK(vert_stiffness);
            molla2->Set_SpringR(vert_damping);
            auto molla2_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
            molla2->AddAsset(molla2_vis);
            mphysicalSystem.Add(molla2);

            auto molla3 = std::make_shared<ChLinkSpring>();
            molla3->Initialize(piastrelle[ix][iz], floorBody, false, ChVector<>( size_tile_x*0.5 + offset_x, tile_gap_y,  size_tile_z*0.5 + offset_z), ChVector<>( size_tile_x*0.5 + offset_x, 0,  size_tile_z*0.5 + offset_z), true);
            molla3->Set_SpringK(vert_stiffness);
            molla3->Set_SpringR(vert_damping);
            auto molla3_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
            molla3->AddAsset(molla3_vis);
            mphysicalSystem.Add(molla3);

            auto molla4 = std::make_shared<ChLinkSpring>();
            molla4->Initialize(piastrelle[ix][iz], floorBody, false, ChVector<>(-size_tile_x*0.5 + offset_x, tile_gap_y,  size_tile_z*0.5 + offset_z), ChVector<>(-size_tile_x*0.5 + offset_x, 0,  size_tile_z*0.5 + offset_z), true);
            molla4->Set_SpringK(vert_stiffness);
            molla4->Set_SpringR(vert_damping);
            auto molla4_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
            molla4->AddAsset(molla4_vis);
            mphysicalSystem.Add(molla4);

            // 4-Create the horizontal springs: soil-tile

            auto mollaHx = std::make_shared<ChLinkSpring>();
            mollaHx->Initialize(piastrelle[ix][iz], floorBody, false, ChVector<>(offset_x, tile_gap_y, offset_z), ChVector<>(offset_x + 0.04, tile_gap_y, offset_z), true);
            mollaHx->Set_SpringK(hor_stiffness);
            mollaHx->Set_SpringR(hor_damping);
            auto mollaHx_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
            mollaHx_vis->SetColor(ChColor(1,0.3,0));
            mollaHx->AddAsset(mollaHx_vis);
            mphysicalSystem.Add(mollaHx);

            auto mollaHy = std::make_shared<ChLinkSpring>();
            mollaHy->Initialize(piastrelle[ix][iz], floorBody, false, ChVector<>(offset_x, tile_gap_y, offset_z), ChVector<>(offset_x, tile_gap_y, 0.04 + offset_z), true);
            mollaHy->Set_SpringK(hor_stiffness);
            mollaHy->Set_SpringR(hor_damping);
            auto mollaHy_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
            mollaHy_vis->SetColor(ChColor(1,0.3,0));
            mollaHy->AddAsset(mollaHy_vis);
            mphysicalSystem.Add(mollaHy);
            */
            
            auto bushing1 = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    floorBody, // body B
                                    ChFrame<>(ChVector<>(-size_tile_x*0.5 + offset_x, tile_gap_y, -size_tile_z*0.5 + offset_z)), //initial frame of bushing in abs space
                                    ChVector<>(hor_stiffness, vert_stiffness, hor_stiffness),    // K stiffness in local frame  [N/m]
                                    ChVector<>(hor_damping, vert_damping, hor_damping),        // R damping in local frame  [N/m/s]
                                    ChVector<>(hor_yeld, vert_yeld, hor_yeld)     // plastic yeld [N/m]
                                    );  
            my_loadcontainer->Add(bushing1);

            auto bushing2 = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    floorBody, // body B
                                    ChFrame<>(ChVector<>( size_tile_x*0.5 + offset_x, tile_gap_y, -size_tile_z*0.5 + offset_z)), //initial frame of bushing in abs space
                                    ChVector<>(hor_stiffness, vert_stiffness, hor_stiffness),    // K stiffness in local frame  [N/m]
                                    ChVector<>(hor_damping, vert_damping, hor_damping),        // R damping in local frame  [N/m/s]
                                    ChVector<>(hor_yeld, vert_yeld, hor_yeld)     // plastic yeld [N/m]
                                    );  
            my_loadcontainer->Add(bushing2);

            auto bushing3 = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    floorBody, // body B
                                    ChFrame<>(ChVector<>( size_tile_x*0.5 + offset_x, tile_gap_y,  size_tile_z*0.5 + offset_z)), //initial frame of bushing in abs space
                                    ChVector<>(hor_stiffness, vert_stiffness, hor_stiffness),    // K stiffness in local frame  [N/m]
                                    ChVector<>(hor_damping, vert_damping, hor_damping),        // R damping in local frame  [N/m/s]
                                    ChVector<>(hor_yeld, vert_yeld, hor_yeld)     // plastic yeld [N/m]
                                    );  
            my_loadcontainer->Add(bushing3);

            auto bushing4 = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    floorBody, // body B
                                    ChFrame<>(ChVector<>(-size_tile_x*0.5 + offset_x, tile_gap_y,  size_tile_z*0.5 + offset_z)), //initial frame of bushing in abs space
                                    ChVector<>(hor_stiffness, vert_stiffness, hor_stiffness),    // K stiffness in local frame  [N/m]
                                    ChVector<>(hor_damping, vert_damping, hor_damping),        // R damping in local frame  [N/m/s]
                                    ChVector<>(hor_yeld, vert_yeld, hor_yeld)     // plastic yeld [N/m]
                                    );  
            my_loadcontainer->Add(bushing4);





            // 5- Create the inter-block springs:

            if (ix>0) {

                auto piastrella_prec_x = piastrelle[ix-1][iz];

                auto bushing_1h = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    piastrella_prec_x,  // body B
                                    ChFrame<>(ChVector<>(offset_x-size_tile_x*0.5, tile_gap_y+size_tile_x*0.5, offset_z+size_tile_z*0.5)), //initial frame of bushing in abs space
                                    ChVector<>(ortho_stiffness_giunto, shear_stiffness_giunto, shear_stiffness_giunto),    // K stiffness in local frame  [N/m]
                                    ChVector<>(ortho_damping_giunto, shear_damping_giunto, shear_damping_giunto),        // R damping in local frame  [N/m/s]
                                    ChVector<>(ortho_yeld_giunto, shear_yeld_giunto, shear_yeld_giunto)     // plastic yeld [N/m]
                                    );  
                my_loadcontainer->Add(bushing_1h);

                auto bushing_2h = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    piastrella_prec_x,  // body B
                                    ChFrame<>(ChVector<>(offset_x-size_tile_x*0.5, tile_gap_y, offset_z-size_tile_z*0.5)), //initial frame of bushing in abs space
                                    ChVector<>(ortho_stiffness_giunto, shear_stiffness_giunto, shear_stiffness_giunto),    // K stiffness in local frame  [N/m]
                                    ChVector<>(ortho_damping_giunto, shear_damping_giunto, shear_damping_giunto),        // R damping in local frame  [N/m/s]
                                    ChVector<>(ortho_yeld_giunto, shear_yeld_giunto, shear_yeld_giunto)     // plastic yeld [N/m]
                                    );  
                my_loadcontainer->Add(bushing_2h);
            }

            if (iz>0) {

                auto piastrella_prec_z = piastrelle[ix][iz-1];

                auto bushing_1h = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    piastrella_prec_z,  // body B
                                    ChFrame<>(ChVector<>(offset_x-size_tile_x*0.5, tile_gap_y, offset_z-size_tile_z*0.5)), //initial frame of bushing in abs space
                                    ChVector<>(shear_stiffness_giunto, shear_stiffness_giunto, ortho_stiffness_giunto),    // K stiffness in local frame  [N/m]
                                    ChVector<>(shear_damping_giunto, shear_damping_giunto, ortho_damping_giunto),        // R damping in local frame  [N/m/s]
                                    ChVector<>(shear_yeld_giunto, shear_yeld_giunto, ortho_yeld_giunto)     // plastic yeld [N/m]
                                    );  
                my_loadcontainer->Add(bushing_1h);

                auto bushing_2h = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    piastrella_prec_z,  // body B
                                    ChFrame<>(ChVector<>(offset_x+size_tile_x*0.5, tile_gap_y, offset_z-size_tile_z*0.5)), //initial frame of bushing in abs space
                                    ChVector<>(shear_stiffness_giunto, shear_stiffness_giunto, ortho_stiffness_giunto),    // K stiffness in local frame  [N/m]
                                    ChVector<>(shear_damping_giunto, shear_damping_giunto, ortho_damping_giunto),        // R damping in local frame  [N/m/s]
                                    ChVector<>(shear_yeld_giunto, shear_yeld_giunto, ortho_yeld_giunto)     // plastic yeld [N/m]
                                    );  
                my_loadcontainer->Add(bushing_2h); 

            }
        }
    }

    // 6- Create a rigid wheel 

    auto wheelBody = std::make_shared<ChBodyEasyCylinder>(0.4, 0.285,  // R, h
                                                        200,         // density
                                                        true,        //  contact geometry
                                                        true          // enable visualization geometry
                                                        );
    wheelBody->SetPos(ChVector<>(0, 0.60, 0));
    wheelBody->SetRot(Q_from_AngX(CH_C_PI_2)); // 90 deg

    mphysicalSystem.Add(wheelBody);


    // 7- apparecchiatura stretcher bond
    
    if (false) {

	   
	    std::shared_ptr<ChBodyEasyBox> piastrelle_shiftate[10][8];

	    for (int iz = 0; iz < 8; iz = iz + 1) {
			    for (int ix = 0; ix < 10; ++ix) {
				    if (iz % 2 == 0) {

				    piastrelle_shiftate[ix][iz] = std::make_shared<ChBodyEasyBox>(size_tile_x, size_tile_y, size_tile_z,
					    tile_density,      // density
					    true,         // ok, contact geometry
					    true          // enable visualization geometry
					    );

				    double offset_x = ix * (size_tile_x + giunto_gap_x);
				    double offset_z = iz * (size_tile_z + giunto_gap_z);

				    piastrelle_shiftate[ix][iz]->SetPos(ChVector<>(
					    -1 - offset_x,
					    size_tile_y*0.5 + tile_gap_y,
					    offset_z
					    ));

				    mphysicalSystem.Add(piastrelle_shiftate[ix][iz]);

				    }
				    else {

                    piastrelle_shiftate[ix][iz] = std::make_shared<ChBodyEasyBox>(size_tile_x, size_tile_y, size_tile_z,
					    tile_density,      // density
					    true,         // ok, contact geometry
					    true          // enable visualization geometry
					    );

				    double offset_x = ix * (size_tile_x + giunto_gap_x);
				    double offset_z = iz * (size_tile_z + giunto_gap_z);

				    piastrelle_shiftate[ix][iz]->SetPos(ChVector<>(
					    -1 - offset_x - 0.5*(size_tile_x + giunto_gap_x),
					    size_tile_y*0.5 + tile_gap_y,
					    offset_z
					    ));

				    mphysicalSystem.Add(piastrelle_shiftate[ix][iz]);
				    }
			    }
	    }

	    for (int iz = 0; iz < 8; ++iz) {
			    for (int ix = 0; ix < 10; ++ix) {
				    if (iz % 2 == 0) {

				    double offset_x = ix * (size_tile_x + giunto_gap_x);
				    double offset_z = iz * (size_tile_z + giunto_gap_z);

				    // 8- Create the vertical springs: soil-piastrella

				    auto spring1 = std::make_shared<ChLinkSpring>();
				    spring1->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(size_tile_x*0.5 - 1 - offset_x, tile_gap_y, -size_tile_z*0.5 + offset_z), ChVector<>(+size_tile_x*0.5 - 1 - offset_x, 0, -size_tile_z*0.5 + offset_z), true);
				    spring1->Set_SpringK(vert_stiffness);
				    spring1->Set_SpringR(vert_damping);
				    auto spring1_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    spring1->AddAsset(spring1_vis);
				    mphysicalSystem.Add(spring1);

				    auto spring2 = std::make_shared<ChLinkSpring>();
				    spring2->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(size_tile_x*0.5 - 1 - offset_x, tile_gap_y, +size_tile_z*0.5 + offset_z), ChVector<>(size_tile_x*0.5 - 1 - offset_x, 0, +size_tile_z*0.5 + offset_z), true);
				    spring2->Set_SpringK(vert_stiffness);
				    spring2->Set_SpringR(vert_damping);
				    auto spring2_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    spring2->AddAsset(spring2_vis);
				    mphysicalSystem.Add(spring2);

				    auto spring3 = std::make_shared<ChLinkSpring>();
				    spring3->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(-size_tile_x*0.5 - 1 - offset_x, tile_gap_y, -size_tile_z*0.5 + offset_z), ChVector<>(-size_tile_x*0.5 - 1 - offset_x, 0, -size_tile_z*0.5 + offset_z), true);
				    spring3->Set_SpringK(vert_stiffness);
				    spring3->Set_SpringR(vert_damping);
				    auto spring3_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    spring3->AddAsset(spring3_vis);
				    mphysicalSystem.Add(spring3);

				    auto spring4 = std::make_shared<ChLinkSpring>();
				    spring4->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(-size_tile_x*0.5 - 1 - offset_x, tile_gap_y, size_tile_z*0.5 + offset_z), ChVector<>(-size_tile_x*0.5 - 1 - offset_x, 0, size_tile_z*0.5 + offset_z), true);
				    spring4->Set_SpringK(vert_stiffness);
				    spring4->Set_SpringR(vert_damping);
				    auto spring4_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    spring4->AddAsset(spring4_vis);
				    mphysicalSystem.Add(spring4);

				    // 9- Create the horizontal springs: soil-piastrella
				
		           /* 
			        quando le inserisco il sistema si muove :(

			        auto springHx = std::make_shared<ChLinkSpring>();
				    springHx->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>( - 1 - offset_x, tile_gap_y, offset_z), ChVector<>( -1 - offset_x + 0.04, tile_gap_y, offset_z), true);
				    springHx->Set_SpringK(hor_stiffness);
				    springHx->Set_SpringR(hor_damping);
				    auto springHx_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    springHx->AddAsset(springHx_vis);
				    mphysicalSystem.Add(springHx);

				    auto springHy = std::make_shared<ChLinkSpring>();
				    springHy->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(- 1 - offset_x, tile_gap_y, offset_z), ChVector<>(-1 - offset_x, tile_gap_y, 0.04 + offset_z), true);
				    springHy->Set_SpringK(hor_stiffness);
				    springHy->Set_SpringR(hor_damping);
				    auto springHy_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    springHy->AddAsset(springHy_vis);
				    mphysicalSystem.Add(springHy);
				    */

				    // 10- Create the inter-block springs (dir x):

				    if (ix > 0) {

					    auto sbpiastrella_prec_x = piastrelle_shiftate[ix - 1][iz];

					    auto spring1h = std::make_shared<ChLinkSpring>();
					    spring1h->Initialize(piastrelle_shiftate[ix][iz], sbpiastrella_prec_x, false, ChVector<>(-1 - offset_x - size_tile_x*0.5 - giunto_gap_x, tile_gap_y, offset_z - size_tile_z*0.5), ChVector<>(-1 - offset_x - size_tile_x*0.5, tile_gap_y, offset_z - size_tile_z*0.5), true);
					    spring1h->Set_SpringK(ortho_stiffness_giunto);
					    spring1h->Set_SpringR(ortho_damping_giunto);
					    auto spring1h_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
					    spring1h_vis->SetColor(ChColor(1, 0, 0));
					    spring1h->AddAsset(spring1h_vis);
					    mphysicalSystem.Add(spring1h);

					    auto spring2h = std::make_shared<ChLinkSpring>();
					    spring2h->Initialize(piastrelle_shiftate[ix][iz], sbpiastrella_prec_x, false, ChVector<>(-1 - offset_x - size_tile_x*0.5 - giunto_gap_x, tile_gap_y, offset_z + size_tile_z*0.5), ChVector<>(-1 - offset_x - size_tile_x*0.5, tile_gap_y, offset_z + size_tile_z*0.5), true);
					    spring2h->Set_SpringK(ortho_stiffness_giunto);
					    spring2h->Set_SpringR(ortho_damping_giunto);
					    auto spring2h_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
					    spring2h_vis->SetColor(ChColor(1, 0, 0));
					    spring2h->AddAsset(spring2h_vis);
					    mphysicalSystem.Add(spring2h);
				    }
				    }
		     else {
			
				    double offset_x = ix * (size_tile_x + giunto_gap_x);
				    double offset_z = iz * (size_tile_z + giunto_gap_z);

				    // 11- Create the vertical springs: soil-piastrella

				    auto spring1 = std::make_shared<ChLinkSpring>();
				    spring1->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(-1 - offset_x - 0.5*giunto_gap_x, tile_gap_y, -size_tile_z*0.5 + offset_z), ChVector<>(-1 - offset_x - 0.5*giunto_gap_x, 0, -size_tile_z*0.5 + offset_z), true);
				    spring1->Set_SpringK(vert_stiffness);
				    spring1->Set_SpringR(vert_damping);
				    auto spring1_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring
				    spring1->AddAsset(spring1_vis);
				    mphysicalSystem.Add(spring1);

				    auto spring2 = std::make_shared<ChLinkSpring>();
				    spring2->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(-1 - offset_x - 0.5*giunto_gap_x, tile_gap_y, +size_tile_z*0.5 + offset_z), ChVector<>(-1 - offset_x - 0.5*giunto_gap_x, 0, +size_tile_z*0.5 + offset_z), true);
				    spring2->Set_SpringK(vert_stiffness);
				    spring2->Set_SpringR(vert_damping);
				    auto spring2_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring
				    spring2->AddAsset(spring2_vis);
				    mphysicalSystem.Add(spring2);

				    auto spring3 = std::make_shared<ChLinkSpring>();
				    spring3->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(-1 - offset_x - 0.5*giunto_gap_x - size_tile_x, tile_gap_y, -size_tile_z*0.5 + offset_z), ChVector<>(-1 - offset_x - 0.5*giunto_gap_x - size_tile_x, 0, -size_tile_z*0.5 + offset_z), true);
				    spring3->Set_SpringK(vert_stiffness);
				    spring3->Set_SpringR(vert_damping);
				    auto spring3_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring
				    spring3->AddAsset(spring3_vis);
				    mphysicalSystem.Add(spring3);

				    auto spring4 = std::make_shared<ChLinkSpring>();
				    spring4->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(-1 - offset_x - 0.5*giunto_gap_x - size_tile_x, tile_gap_y, size_tile_z*0.5 + offset_z), ChVector<>(-1 - offset_x - 0.5*giunto_gap_x - size_tile_x, 0, size_tile_z*0.5 + offset_z), true);
				    spring4->Set_SpringK(vert_stiffness);
				    spring4->Set_SpringR(vert_damping);
				    auto spring4_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring
				    spring4->AddAsset(spring4_vis);
				    mphysicalSystem.Add(spring4);

				    // 12- Create the horizontal springs: soil-piastrella
				
			        /*
				    Quando le inserisco il sistema si muove :(

				    auto springHx = std::make_shared<ChLinkSpring>();
				    springHx->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(-1 - offset_x - 0.5*(giunto_gap_x + size_tile_x), tile_gap_y, offset_z), ChVector<>(-1 - offset_x - 0.5*(giunto_gap_x + size_tile_x) + 0.04, tile_gap_y, offset_z), true);
				    springHx->Set_SpringK(hor_stiffness);
				    springHx->Set_SpringR(hor_damping);
				    auto springHx_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    springHx->AddAsset(springHx_vis);
				    mphysicalSystem.Add(springHx);

				    auto springHy = std::make_shared<ChLinkSpring>();
				    springHy->Initialize(piastrelle_shiftate[ix][iz], floorBody, false, ChVector<>(-1 - offset_x - 0.5*(giunto_gap_x + size_tile_x), tile_gap_y, offset_z), ChVector<>(-1 - offset_x - 0.5*(giunto_gap_x + size_tile_x), tile_gap_y, 0.04 + offset_z), true);
				    springHy->Set_SpringK(hor_stiffness);
				    springHy->Set_SpringR(hor_damping);
				    auto springHy_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    springHy->AddAsset(springHy_vis);
				    mphysicalSystem.Add(springHy);
				    */

				    // 13- Create the inter-block springs (dir x):

				    if (ix>0) {

					    auto sbpiastrella_prec_x = piastrelle_shiftate[ix - 1][iz];

					    auto spring1h = std::make_shared<ChLinkSpring>();
					    spring1h->Initialize(piastrelle_shiftate[ix][iz], sbpiastrella_prec_x, false, ChVector<>(-1 - offset_x - size_tile_x - giunto_gap_x - giunto_gap_x*0.5, tile_gap_y, offset_z - size_tile_z*0.5), ChVector<>(-1 - offset_x - size_tile_x - giunto_gap_x*0.5, tile_gap_y, offset_z - size_tile_z*0.5), true);
					    spring1h->Set_SpringK(ortho_stiffness_giunto);
					    spring1h->Set_SpringR(ortho_damping_giunto);
					    auto spring1h_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
					    spring1h_vis->SetColor(ChColor(1, 0, 0));
					    spring1h->AddAsset(spring1h_vis);
					    mphysicalSystem.Add(spring1h);

					    auto spring2h = std::make_shared<ChLinkSpring>();
					    spring2h->Initialize(piastrelle_shiftate[ix][iz], sbpiastrella_prec_x, false, ChVector<>(-1 - offset_x - size_tile_x - giunto_gap_x - giunto_gap_x*0.5, tile_gap_y, offset_z + size_tile_z*0.5), ChVector<>(-1 - offset_x - size_tile_x - giunto_gap_x*0.5, tile_gap_y, offset_z + size_tile_z*0.5), true);
					    spring2h->Set_SpringK(ortho_stiffness_giunto);
					    spring2h->Set_SpringR(ortho_damping_giunto);
					    auto spring2h_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
					    spring2h_vis->SetColor(ChColor(1, 0, 0));
					    spring2h->AddAsset(spring2h_vis);
					    mphysicalSystem.Add(spring2h);
				    }
			    }
	        }
         }

        // 14-Create the inter-block springs (dir z file pari):

	    for (int iz = 2; iz < 8; iz = iz + 2) {
		    for (int ix = 0; ix < 10; ++ix) {
		
				    double offset_x = ix * (size_tile_x + giunto_gap_x);
				    double offset_z = iz * (size_tile_z + giunto_gap_z);

				    auto sb_piastrella_pre_z = piastrelle_shiftate[ix][iz - 1];

				    auto spring1v = std::make_shared<ChLinkSpring>();
				    spring1v->Initialize(piastrelle_shiftate[ix][iz], sb_piastrella_pre_z, false, ChVector<>(-1 - offset_x - 0.5*size_tile_x, tile_gap_y, offset_z - size_tile_z*0.5), ChVector<>(-1 - offset_x - 0.5*size_tile_x, tile_gap_y, offset_z - size_tile_z*0.5 - giunto_gap_z), true);
				    spring1v->Set_SpringK(ortho_stiffness_giunto);
				    spring1v->Set_SpringR(ortho_damping_giunto);
				    auto spring1v_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring
				    spring1v->AddAsset(spring1v_vis);
				    spring1v_vis->SetColor(ChColor(0, 1, 0));
				    mphysicalSystem.Add(spring1v);

			    if (ix > 0) {

				    double offset_x = ix * (size_tile_x + giunto_gap_x);
				    double offset_z = iz * (size_tile_z + giunto_gap_z);

				    auto sb_piastrella_pre_x_pre_z = piastrelle_shiftate[ix - 1][iz - 1];

				    auto spring2v = std::make_shared<ChLinkSpring>();
				    spring2v->Initialize(piastrelle_shiftate[ix][iz], sb_piastrella_pre_x_pre_z, false, ChVector<>(-1 - offset_x + 0.5*size_tile_x, tile_gap_y, offset_z - size_tile_z*0.5), ChVector<>(-1 - offset_x + 0.5*size_tile_x, tile_gap_y, offset_z - size_tile_z*0.5 - giunto_gap_z), true);
				    spring2v->Set_SpringK(ortho_stiffness_giunto);
				    spring2v->Set_SpringR(ortho_damping_giunto);
				    auto spring2v_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring
				    spring2v->AddAsset(spring2v_vis);
				    spring2v_vis->SetColor(ChColor(0, 1, 0));
				    mphysicalSystem.Add(spring2v);
			    }
		    }
	    }

	    // 15-Create the inter-block springs (dir z file dispari):

	    for (int iz = 1; iz < 8; iz = iz + 2) {
		    for (int ix = 0; ix < 10; ++ix) {

			    double offset_x = ix * (size_tile_x + giunto_gap_x);
			    double offset_z = iz * (size_tile_z + giunto_gap_z);

			    auto sb_piastrella_pre_z = piastrelle_shiftate[ix][iz - 1];

			    auto spring1v = std::make_shared<ChLinkSpring>();
			    spring1v->Initialize(piastrelle_shiftate[ix][iz], sb_piastrella_pre_z, false, ChVector<>(-1 - offset_x - 0.5*size_tile_x, tile_gap_y, offset_z - size_tile_z*0.5), ChVector<>(-1 - offset_x - 0.5*size_tile_x, tile_gap_y, offset_z - size_tile_z*0.5 - giunto_gap_z), true);
			    spring1v->Set_SpringK(ortho_stiffness_giunto);
			    spring1v->Set_SpringR(ortho_damping_giunto);
			    auto spring1v_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring
			    spring1v->AddAsset(spring1v_vis);
			    spring1v_vis->SetColor(ChColor(0, 1, 0));
			    mphysicalSystem.Add(spring1v);

			    if (ix > 0) {

			    double offset_x = ix * (size_tile_x + giunto_gap_x);
			    double offset_z = iz * (size_tile_z + giunto_gap_z);

			    auto sb_piastrella_pre_z = piastrelle_shiftate[ix][iz - 1];

			    auto spring2v = std::make_shared<ChLinkSpring>();
			    spring2v->Initialize(piastrelle_shiftate[ix-1][iz], sb_piastrella_pre_z, false, ChVector<>(-1 - offset_x + 0.5*size_tile_x, tile_gap_y, offset_z - size_tile_z*0.5), ChVector<>(-1 - offset_x + 0.5*size_tile_x, tile_gap_y, offset_z - size_tile_z*0.5 - giunto_gap_z), true);
			    spring2v->Set_SpringK(ortho_stiffness_giunto);
			    spring2v->Set_SpringR(ortho_damping_giunto);
			    auto spring2v_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring
			    spring2v->AddAsset(spring2v_vis);
			    spring2v_vis->SetColor(ChColor(0, 1, 0));
			    mphysicalSystem.Add(spring2v);
			
			    }	
		    }
	    }
	
	    /*
	    //Se inserisco la ruota, per quanto leggera, mi fa esplodere il sistema (immagino per mancanza di contrasto laterale delle piastrelle )
	    // 16- Create a rigid wheel 

	    auto sb_wheelBody = std::make_shared<ChBodyEasyCylinder>(0.4, 0.285,  // R, h
															    200,         // density
															    true,        //  contact geometry
															    true          // enable visualization geometry
															    );
	    sb_wheelBody->SetPos(ChVector<>(-2.5, 1, 0.5));
	    sb_wheelBody->SetRot(Q_from_AngX(CH_C_PI_2)); // 90 deg

	    mphysicalSystem.Add(sb_wheelBody);
	    */


    } // end apparecchiatura stretcher bond
	


	// 17-apparecchiatura a spina di pesce
	// imporre condizioni: giunto_gap_x=giunto_gap_z, 2*size_tile_z+giunto_gap_x=size_tile_x
	// l'indice iz deve essere 4, non deve essere cambiato, individua il blocco tipo che si ripete ix volte in dir x e izz volte in dir z

    if (false) {

	    std::shared_ptr<ChBodyEasyBox> piastrelle_herringbone_h[6][5][4];

	    //  18-piastrelle orizzontali apparecchiatura a spina di pesce

	    for (int iz = 0; iz < 4; ++iz) {
	      for (int izz = 0; izz < 5; ++izz) {
			    for (int ix = 0; ix < 6; ++ix) {

				    piastrelle_herringbone_h[ix][izz][iz] = std::make_shared<ChBodyEasyBox>(size_tile_x, size_tile_y, size_tile_z,
																							    tile_density,      // density
																							    true,         // ok, contact geometry
																							    true          // enable visualization geometry
																							    );
				    double offset_x = ix * (size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z) + iz * (size_tile_z + giunto_gap_x);
				    double offset_z = iz * (size_tile_z + giunto_gap_z);

				    piastrelle_herringbone_h[ix][izz][iz]->SetPos(ChVector<>(
															    0.5*size_tile_x  +  offset_x,
															    size_tile_y*0.5  + tile_gap_y,
															    -1 - 0.5*size_tile_z - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)
															    ));

				    mphysicalSystem.Add(piastrelle_herringbone_h[ix][izz][iz]);
			    }
		    }
	    }
	
	    //  19-piastrelle verticali apparecchiatura a spina di pesce

	    std::shared_ptr<ChBodyEasyBox> piastrelle_herringbone_v[6][5][4];

	    for (int iz = 0; iz < 4; ++iz) {
		    for (int izz = 0; izz < 5; ++izz) {
		         for (int ix = 0; ix < 6; ++ix) {
	       
				    piastrelle_herringbone_v[ix][izz][iz] = std::make_shared<ChBodyEasyBox>(size_tile_z, size_tile_y, size_tile_x,
																						    tile_density,      // density
																						    true,         // ok, contact geometry
																						    true          // enable visualization geometry
																						    );
				    double offset_x = ix * (size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z) + iz * (size_tile_z + giunto_gap_x);
				    double offset_z = iz * (size_tile_z + giunto_gap_z);

				    piastrelle_herringbone_v[ix][izz][iz]->SetPos(ChVector<>(
												    0.5*size_tile_z + offset_x,
												    size_tile_y*0.5 + tile_gap_y,
												    -1 - (size_tile_z + giunto_gap_z + 0.5*size_tile_x) - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)
												    ));

				    mphysicalSystem.Add(piastrelle_herringbone_v[ix][izz][iz]);
			    }
		    }
	    }

	    // 20-Create the vertical springs: soil-piastrella (piastrelle orizzontali)

	    for (int iz = 0; iz < 4; ++iz) {
	       for (int izz = 0; izz < 5; ++izz) {
			    for (int ix = 0; ix < 6; ++ix) {
	
			    double offset_x = ix * (size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z) + iz * (size_tile_z + giunto_gap_x);
			    double offset_z = iz * (size_tile_z + giunto_gap_z);

			    auto hb_spring1 = std::make_shared<ChLinkSpring>();
			    hb_spring1->Initialize(piastrelle_herringbone_h[ix][izz][iz], floorBody, false, ChVector<>(offset_x, tile_gap_y, -1 - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), ChVector<>(offset_x, 0, -1 - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), true);
			    hb_spring1->Set_SpringK(vert_stiffness);
			    hb_spring1->Set_SpringR(vert_damping);
			    auto hb_spring1_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
			    hb_spring1->AddAsset(hb_spring1_vis);
			    mphysicalSystem.Add(hb_spring1);
			
			    auto hb_spring2 = std::make_shared<ChLinkSpring>();
			    hb_spring2->Initialize(piastrelle_herringbone_h[ix][izz][iz], floorBody, false, ChVector<>(offset_x + size_tile_x, tile_gap_y, -1 - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), ChVector<>(offset_x + size_tile_x, 0, -1 - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), true);
			    hb_spring2->Set_SpringK(vert_stiffness);
			    hb_spring2->Set_SpringR(vert_damping);
			    auto hb_spring2_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
			    hb_spring2->AddAsset(hb_spring2_vis);
			    mphysicalSystem.Add(hb_spring2);

			    auto hb_spring3 = std::make_shared<ChLinkSpring>();
			    hb_spring3->Initialize(piastrelle_herringbone_h[ix][izz][iz], floorBody, false, ChVector<>(offset_x, tile_gap_y, -1 - offset_z - size_tile_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), ChVector<>(offset_x, 0, -1 - offset_z -size_tile_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), true);
			    hb_spring3->Set_SpringK(vert_stiffness);
			    hb_spring3->Set_SpringR(vert_damping);
			    auto hb_spring3_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
			    hb_spring3->AddAsset(hb_spring3_vis);
			    mphysicalSystem.Add(hb_spring3);
			
			    auto hb_spring4 = std::make_shared<ChLinkSpring>();
			    hb_spring4->Initialize(piastrelle_herringbone_h[ix][izz][iz], floorBody, false, ChVector<>(offset_x + size_tile_x, tile_gap_y, -1 - offset_z - size_tile_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), ChVector<>(offset_x + size_tile_x, 0, -1 - offset_z - size_tile_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), true);
			    hb_spring4->Set_SpringK(vert_stiffness);
			    hb_spring4->Set_SpringR(vert_damping);
			    auto hb_spring4_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
			    hb_spring4->AddAsset(hb_spring4_vis);
			    mphysicalSystem.Add(hb_spring4);	
			    }
		    }
	    }

	    // 21-Create the vertical springs: soil-piastrella (piastrelle verticali)

	    for (int iz = 0; iz < 4; ++iz) {
		    for (int izz = 0; izz < 5; ++izz) {
			    for (int ix = 0; ix < 6; ++ix) {

				    double offset_x = ix * (size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z) + iz * (size_tile_z + giunto_gap_x);
				    double offset_z = iz * (size_tile_z + giunto_gap_z);

				    auto hb_spring1 = std::make_shared<ChLinkSpring>();
				    hb_spring1->Initialize(piastrelle_herringbone_v[ix][izz][iz], floorBody, false, ChVector<>(offset_x, tile_gap_y, -1 - (size_tile_z + giunto_gap_z) - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), ChVector<>(offset_x, 0, -1 - (size_tile_z + giunto_gap_z) - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), true);
				    hb_spring1->Set_SpringK(vert_stiffness);
				    hb_spring1->Set_SpringR(vert_damping);
				    auto hb_spring1_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    hb_spring1->AddAsset(hb_spring1_vis);
				    mphysicalSystem.Add(hb_spring1);

				    auto hb_spring2 = std::make_shared<ChLinkSpring>();
				    hb_spring2->Initialize(piastrelle_herringbone_v[ix][izz][iz], floorBody, false, ChVector<>(offset_x + size_tile_z, tile_gap_y, -1 - (size_tile_z + giunto_gap_z) - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), ChVector<>(offset_x + size_tile_z, 0, -1 - (size_tile_z + giunto_gap_z) - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), true);
				    hb_spring2->Set_SpringK(vert_stiffness);
				    hb_spring2->Set_SpringR(vert_damping);
				    auto hb_spring2_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    hb_spring2->AddAsset(hb_spring2_vis);
				    mphysicalSystem.Add(hb_spring2);

				    auto hb_spring3 = std::make_shared<ChLinkSpring>();
				    hb_spring3->Initialize(piastrelle_herringbone_v[ix][izz][iz], floorBody, false, ChVector<>(offset_x, tile_gap_y, -1 - (size_tile_z + giunto_gap_z + size_tile_x) - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), ChVector<>(offset_x, 0, -1 - (size_tile_z + giunto_gap_z + size_tile_x) - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), true);
				    hb_spring3->Set_SpringK(vert_stiffness);
				    hb_spring3->Set_SpringR(vert_damping);
				    auto hb_spring3_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    hb_spring3->AddAsset(hb_spring3_vis);
				    mphysicalSystem.Add(hb_spring3);

				    auto hb_spring4 = std::make_shared<ChLinkSpring>();
				    hb_spring4->Initialize(piastrelle_herringbone_v[ix][izz][iz], floorBody, false, ChVector<>(offset_x + size_tile_z, tile_gap_y, -1 - (size_tile_z + giunto_gap_z +size_tile_x) - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), ChVector<>(offset_x + size_tile_z, 0, -1 - (size_tile_z + giunto_gap_z +size_tile_x) - offset_z - izz*(size_tile_x + 3 * giunto_gap_x + 2 * size_tile_z)), true);
				    hb_spring4->Set_SpringK(vert_stiffness);
				    hb_spring4->Set_SpringR(vert_damping);
				    auto hb_spring4_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
				    hb_spring4->AddAsset(hb_spring4_vis);
				    mphysicalSystem.Add(hb_spring4);
			    }
		    }
	    }

	    // 22- Create a rigid wheel 

	    auto hb_wheelBody = std::make_shared<ChBodyEasyCylinder>(0.4, 0.2,  // R, h
		    200,         // density
		    true,        //  contact geometry
		    true          // enable visualization geometry
		    );
	    hb_wheelBody->SetPos(ChVector<>(2,size_tile_y + tile_gap_y + 0.1,-2));

	    mphysicalSystem.Add(hb_wheelBody);
     
    } // end apparecchiatura spinapesce 
	

    // Optionally, attach a RGB color asset to the floor, for better visualization
    auto color = std::make_shared<ChColorAsset>();
    color->SetColor(ChColor(0.2f, 0.25f, 0.25f));
    floorBody->AddAsset(color);

    /*
    // Optionally, attach a texture to the pendulum, for better visualization
    auto texture = std::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("cubetexture_bluwhite.png"));  // texture in ../data
    pendulumBody->AddAsset(texture);
    */



    //======================================================================

    // Use this function for adding a ChIrrNodeAsset to all items
    // Otherwise use application.AssetBind(myitem); on a per-item basis.
    application.AssetBindAll();

    // Use this function for 'converting' assets into Irrlicht meshes
    application.AssetUpdateAll();

    // Adjust some settings:
    application.SetTimestep(0.002);
    application.SetTryRealtime(true);

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // This performs the integration timestep!
        application.DoStep();

        application.EndScene();
    }

    return 0;
}
