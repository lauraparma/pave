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
    application.AddTypicalCamera(core::vector3df(2, 2, -5),
                                 core::vector3df(0, 1, 0));  // to change the position of camera
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


    // Some parameters:

    double size_tile_x = 0.3;
    double size_tile_y = 0.1;
    double size_tile_z = 0.2;
    double tile_density = 1500; 
    double tile_gap_y = 0.05; // respect to soil
    double giunto_gap_x = 0.01;
    double giunto_gap_z = 0.01;
    // springs: vertical, soil:
    double vert_stiffness = 12500;  // N/m
    double vert_damping   = 100;    // N/ m/s
    // springs: horizontal, soil:
    double hor_stiffness = 12500;  // N/m
    double hor_damping   = 100;    // N/ m/s
    // springs: horizontal, soil:
    double hor_stiffness_giunto = 10500;  // N/m
    double hor_damping_giunto   = 100;    // N/ m/s

    std::shared_ptr<ChBodyEasyBox> piastrelle[10][8];

    for (int iz =  0; iz < 8; ++iz) {
        for (int ix =  0; ix < 10; ++ix) {

            // 2-Create a piastrella

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

            // Create the vertical springs: soil-piastrella

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

            // Create the horizontal springs: soil-piastrella

            auto mollaHx = std::make_shared<ChLinkSpring>();
            mollaHx->Initialize(piastrelle[ix][iz], floorBody, false, ChVector<>(offset_x, tile_gap_y, offset_z), ChVector<>(offset_x + 0.04, tile_gap_y, offset_z), true);
            mollaHx->Set_SpringK(hor_stiffness);
            mollaHx->Set_SpringR(hor_damping);
            auto mollaHx_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
            mollaHx->AddAsset(mollaHx_vis);
            mphysicalSystem.Add(mollaHx);

            auto mollaHy = std::make_shared<ChLinkSpring>();
            mollaHy->Initialize(piastrelle[ix][iz], floorBody, false, ChVector<>(offset_x, tile_gap_y, offset_z), ChVector<>(offset_x, tile_gap_y, 0.04 + offset_z), true);
            mollaHy->Set_SpringK(hor_stiffness);
            mollaHy->Set_SpringR(hor_damping);
            auto mollaHy_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
            mollaHy->AddAsset(mollaHy_vis);
            mphysicalSystem.Add(mollaHy);

            // Create the inter-block springs:
            if (ix>0) {
                auto piastrella_prec_x = piastrelle[ix-1][iz];

                auto molla1h = std::make_shared<ChLinkSpring>();
                molla1h->Initialize(piastrelle[ix][iz], piastrella_prec_x, false, ChVector<>(offset_x-size_tile_x*0.5, tile_gap_y, offset_z+size_tile_z*0.5), ChVector<>(offset_x-size_tile_x*0.5 -giunto_gap_x, tile_gap_y, offset_z+size_tile_z*0.5), true);
                molla1h->Set_SpringK(hor_stiffness_giunto);
                molla1h->Set_SpringR(hor_damping_giunto);
                auto molla1h_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
                molla1h_vis->SetColor(ChColor(1,0,0));
                molla1h->AddAsset(molla1h_vis);
                mphysicalSystem.Add(molla1h);

                auto molla2h = std::make_shared<ChLinkSpring>();
                molla2h->Initialize(piastrelle[ix][iz], piastrella_prec_x, false, ChVector<>(offset_x-size_tile_x*0.5, tile_gap_y, offset_z-size_tile_z*0.5), ChVector<>(offset_x-size_tile_x*0.5 -giunto_gap_x, tile_gap_y, offset_z-size_tile_z*0.5), true);
                molla2h->Set_SpringK(hor_stiffness_giunto);
                molla2h->Set_SpringR(hor_damping_giunto);
                auto molla2h_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
                molla2h_vis->SetColor(ChColor(1,0,0));
                molla2h->AddAsset(molla2h_vis);
                mphysicalSystem.Add(molla2h);
            }
            if (iz>0) {
                auto piastrella_prec_z = piastrelle[ix][iz-1];

                auto molla1h = std::make_shared<ChLinkSpring>();
                molla1h->Initialize(piastrelle[ix][iz], piastrella_prec_z, false, ChVector<>(offset_x-size_tile_x*0.5, tile_gap_y, offset_z-size_tile_z*0.5), ChVector<>(offset_x-size_tile_x*0.5, tile_gap_y, offset_z-size_tile_z*0.5 -giunto_gap_z), true);
                molla1h->Set_SpringK(hor_stiffness_giunto);
                molla1h->Set_SpringR(hor_damping_giunto);
                auto molla1h_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
                molla1h->AddAsset(molla1h_vis);
                molla1h_vis->SetColor(ChColor(0,1,0));
                mphysicalSystem.Add(molla1h);

                auto molla2h = std::make_shared<ChLinkSpring>();
                molla2h->Initialize(piastrelle[ix][iz], piastrella_prec_z, false, ChVector<>(offset_x+size_tile_x*0.5, tile_gap_y, offset_z-size_tile_z*0.5), ChVector<>(offset_x+size_tile_x*0.5, tile_gap_y, offset_z-size_tile_z*0.5 -giunto_gap_z), true);
                molla2h->Set_SpringK(hor_stiffness_giunto);
                molla2h->Set_SpringR(hor_damping_giunto);
                auto molla2h_vis = std::make_shared<ChPointPointSegment>();  // or..  ChPointPointSpring 
                molla2h_vis->SetColor(ChColor(0,1,0));
                molla2h->AddAsset(molla2h_vis);
                mphysicalSystem.Add(molla2h);

            }




        }
    }



    // 3- Create a rigid wheel 

    auto wheelBody = std::make_shared<ChBodyEasyCylinder>(0.4, 0.285,  // R, h
                                                        200,         // density
                                                        true,        //  contact geometry
                                                        true          // enable visualization geometry
                                                        );
    wheelBody->SetPos(ChVector<>(0, 0.60, 0));
    wheelBody->SetRot(Q_from_AngX(CH_C_PI_2)); // 90 deg

    mphysicalSystem.Add(wheelBody);


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
    application.SetTimestep(0.005);
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
