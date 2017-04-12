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
#include "chrono/core/ChFileutils.h"
#include "chrono/core/ChFrame.h"
#include "core/ChMath.h"

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




class ChLoadBodyBodyBushingPlasticBreak : public ChLoadBodyBodyBushingSpherical {
protected:
    ChVector<> yeld;
    ChVector<> plastic_def;
    ChVector<> def_limit_break;
    bool broken;
   
public:
    ChLoadBodyBodyBushingPlasticBreak(
                          std::shared_ptr<ChBody> mbodyA,   ///< object A
                          std::shared_ptr<ChBody> mbodyB,   ///< object B
                          const ChFrame<> abs_application,  ///< create the bushing here, in abs. coordinates. Initial alignment as world xyz.
                          const ChVector<> mstiffness,      ///< stiffness, along x y z axes of the abs_application
                          const ChVector<> mdamping,        ///< damping, along x y z axes of the abs_application
                          const ChVector<> myeld,           ///< plastic yeld, along x y z axes of the abs_application
                          const ChVector<> mdef_limit       ///< deformation limit, along x y z axes: if surpassed -> go into broken  state
                        ) 
         : ChLoadBodyBodyBushingSpherical(mbodyA,mbodyB,abs_application, mstiffness, mdamping), 
           yeld(myeld),
           def_limit_break(mdef_limit),
           broken (false),
           plastic_def(VNULL) {         
        }

        /// Implement the computation of bushing force, in local 
        /// coordinates of the loc_application_B.
        /// Force is assumed applied to body B, and its opposite to A.
    virtual void ComputeBushingForceTorque(const ChFrameMoving<>& rel_AB, 
                                            ChVector<>& loc_force,
                                            ChVector<>& loc_torque)  override {
        if (fabs(rel_AB.GetPos().x()) >= def_limit_break.x() ||
            fabs(rel_AB.GetPos().y()) >= def_limit_break.y() ||
            fabs(rel_AB.GetPos().z()) >= def_limit_break.z() ) {

            broken = true;
        }

        if (broken) {
            loc_force = VNULL;
            loc_torque = VNULL;
            return;
        }

        loc_force  = (rel_AB.GetPos()-plastic_def)  * this->stiffness  // element-wise product!
                   + rel_AB.GetPos_dt() * this->damping;   // element-wise product!

        // A basic plasticity, assumed with box capping, without hardening:
        
        if (loc_force.x() > yeld.x()) {
            loc_force.x() = yeld.x();
            plastic_def.x() = rel_AB.GetPos().x() - loc_force.x() / this->stiffness.x();
        }
        if (loc_force.x() < -yeld.x()) {
            loc_force.x() = -yeld.x();
            plastic_def.x() = rel_AB.GetPos().x() - loc_force.x() / this->stiffness.x();
        }
        if (loc_force.y() > yeld.y()) {
            loc_force.y() = yeld.y();
            plastic_def.y() = rel_AB.GetPos().y() - loc_force.y() / this->stiffness.y();
        }
        if (loc_force.y() < -yeld.y()) {
            loc_force.y() = -yeld.y();
            plastic_def.y() = rel_AB.GetPos().y() - loc_force.y() / this->stiffness.y();
        }
        if (loc_force.z() > yeld.z()) {
            loc_force.z() = yeld.z();
            plastic_def.z() = rel_AB.GetPos().z() - loc_force.z() / this->stiffness.z();
        }
        if (loc_force.z() < -yeld.z()) {
            loc_force.z() = -yeld.z();
            plastic_def.z() = rel_AB.GetPos().z() - loc_force.z() / this->stiffness.z();
        }
        
        // GetLog() << "loc_force" << loc_force << "\n";
        // GetLog() << "plastic_def" << plastic_def << "\n";
        loc_torque = VNULL;
    }

    virtual bool IsStiff() {return true;}


        /// Set plastic yeld, forces beyond this limit will be capped. 
        /// Expressed along the x y z axes of loc_application_B, es [N/m].
    void SetYeld(const ChVector<> myeld) {this->yeld = myeld;}
    ChVector<> GetYeld() const {return this->yeld;}

        /// Get the current accumulated plastic deformation, in [m], that
        /// could become nonzero if forces went beyond the plastic yeld.
    ChVector<> GetPlasticDeformation() const {return this->plastic_def;}

};




// This is the contact reporter class, just for writing contacts on 
// a file on disk
class _contact_reporter_class : public  chrono::ChReportContactCallback 
{
    public:
    ChStreamOutAsciiFile* mfile; // the file to save data into

    virtual bool ReportContactCallback(
                                const ChVector<>& pA,             ///< get contact pA
                                const ChVector<>& pB,             ///< get contact pB
                                const ChMatrix33<>& plane_coord,  ///< get contact plane coordsystem (A column 'X' is contact normal)
                                const double& distance,           ///< get contact distance
                                const ChVector<>& react_forces,   ///< get react.forces (if already computed). In coordsystem 'plane_coord'
                                const ChVector<>& react_torques,  ///< get react.torques, if rolling friction (if already computed).
                                ChContactable* contactobjA,  ///< get model A (note: some containers may not support it and could be zero!)
                                ChContactable* contactobjB   ///< get model B (note: some containers may not support it and could be zero!)
        )
    {
        // For each contact, this function is executed. 
        // In this example, saves on ascii file:
        //   bodyA ID, bodyB ID, position x,y,z, force normal,tangentU,tangentV, rotation matrix (9 values: normal xyz, tangent xyz, tangent xyz)
        (*mfile)    << contactobjA->GetPhysicsItem()->GetIdentifier() << ", "
                    << contactobjB->GetPhysicsItem()->GetIdentifier() << ", "    
                    << pA.x() << ", " 
                    << pA.y() << ", " 
                    << pA.z() << ", " 
                    << react_forces.x() << ", "
                    << react_forces.y() << ", "
                    << react_forces.z() << ", "
                    << plane_coord.Get_A_Xaxis().x() << ", "
                    << plane_coord.Get_A_Xaxis().y() << ", "
                    << plane_coord.Get_A_Xaxis().z() << ", "
                    << plane_coord.Get_A_Yaxis().x() << ", "
                    << plane_coord.Get_A_Yaxis().y() << ", "
                    << plane_coord.Get_A_Yaxis().z() << ", "
                    << plane_coord.Get_A_Zaxis().x() << ", "
                    << plane_coord.Get_A_Zaxis().y() << ", "
                    << plane_coord.Get_A_Zaxis().z() << "\n";
        return true;  // to continue scanning contacts
    }
};





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
    application.AddTypicalCamera(core::vector3df(-4, 3, -4),
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

	// archi contrastanti (inizio)

	double raggio = 0.8;
    double lato_blocco = 0.1;  // lato cubetto
	double tile_gap_y = 0.05; // respect to soil
	double l_giunto_azimut = 0.01; // larghezza giunto lungo azimut (valore minimo)
	double l_giunto_radiale = 0.01; // larghezza giunto long.

	//  blocchi sottosquadra (inizio)

	std:: vector<ChVector<> > mpoints; // punti dell'inviluppo convesso blocco sottosquadra (rapporto lato sup/lat inf = 0.4)

	for (int i = 0; i < 4; ++i) {
		double alpha = CH_C_PI_4 + i*CH_C_PI_2;  // polar coord
		double x = CH_C_SQRT_2*0.5*lato_blocco*cos(alpha);
		double z = CH_C_SQRT_2*0.5*lato_blocco*sin(alpha);
		double y = lato_blocco;
		mpoints.push_back(ChVector<>(x, y, z));
	}

	for (int i = 0; i < 4; ++i) {
		double alpha = CH_C_PI_4 + i*CH_C_PI_2;  // polar coord
		double x = CH_C_SQRT_2*0.5*lato_blocco*0.4*cos(alpha);
		double z = CH_C_SQRT_2*0.5*lato_blocco*0.4*sin(alpha);
		double y = 0;
		mpoints.push_back(ChVector<>(x, y, z));
	}


	// arco sinistra blocchi sottosquadra

	std::shared_ptr<ChBodyEasyConvexHull> blocco_sottosquadra_s[7];

	for (int ix = 0; ix < 7; ++ix) {

		blocco_sottosquadra_s[ix] = std::make_shared<ChBodyEasyConvexHull>(mpoints, 1500, true, true);

		double offset_azimut = (ix - 1)*(lato_blocco / raggio + l_giunto_azimut / raggio);

		blocco_sottosquadra_s[ix]->SetPos(ChVector<>(-1 - (raggio + 0.5*lato_blocco)*sin(offset_azimut), tile_gap_y + lato_blocco*(double(2) / double(3)), (raggio + 0.5*lato_blocco)*cos(offset_azimut)));
		blocco_sottosquadra_s[ix]->SetRot(Q_from_AngY(-offset_azimut));
		blocco_sottosquadra_s[ix]->SetBodyFixed(true);

		mphysicalSystem.Add(blocco_sottosquadra_s[ix]);
	}

	
	// arco destra blocchi sottosquadra (funziona con un solo blocchetto!!!? perché?


	std::shared_ptr<ChBodyEasyConvexHull> blocco_sottosquadra_d[1];

	for (int ix = 0; ix < 1; ++ix) {

		blocco_sottosquadra_d[ix] = std::make_shared<ChBodyEasyConvexHull>(mpoints, 1500, true, true);

		double offset_azimut = (ix - 1)*(lato_blocco/raggio + l_giunto_azimut/raggio);

		blocco_sottosquadra_d[ix]->SetPos(ChVector<>(-1 + (raggio + 0.5*lato_blocco)*sin(offset_azimut), tile_gap_y + lato_blocco*(double(2) / double(3)), (raggio + 0.5*lato_blocco)*cos(offset_azimut)));
		blocco_sottosquadra_d[ix]->SetRot(Q_from_AngY(offset_azimut));
		blocco_sottosquadra_d[ix]->SetBodyFixed(true);

		mphysicalSystem.Add(blocco_sottosquadra_d[ix]);
	}
	
    // archi contrastanti con blocchi sottosquadra (fine)

	// archi contrastanti con blocchi normali (inizio)
	// arco sinistra

	std::shared_ptr<ChBodyEasyBox> blocchetto_s[6];

	for (int ix = 0; ix < 6; ++ix) {

		blocchetto_s[ix] = std::make_shared<ChBodyEasyBox>(lato_blocco, lato_blocco, lato_blocco,
															1500,      // density
															true,         // ok, contact geometry
															true          // enable visualization geometry
															);

		double offset_azimut = (ix - 1)*(lato_blocco/raggio + l_giunto_azimut/raggio);

		blocchetto_s[ix]->SetPos(ChVector<>(-1 - (raggio + 0.5*lato_blocco)*sin(offset_azimut), tile_gap_y + 0.5*lato_blocco, -1 + (raggio + 0.5*lato_blocco)*cos(offset_azimut)));
		blocchetto_s[ix]->SetRot(Q_from_AngY(-offset_azimut));
		blocchetto_s[ix]->SetBodyFixed(true);

		mphysicalSystem.Add(blocchetto_s[ix]);
	}

	// blocco condiviso (non funziona!)
	/*
	double lato_blocco_cond = lato_blocco*CH_C_SQRT_2*0.5;
	auto blocchetto_condiv = std::make_shared<ChBodyEasyBox>(lato_blocco_cond, lato_blocco_cond, lato_blocco_cond,
												1500,      // density
												false,         // ok, contact geometry
												true          // enable visualization geometry
												);
	blocchetto_condiv->SetPos(ChVector<>(-1 - (raggio + lato_blocco)*CH_C_SQRT_2*0.5, tile_gap_y + 0.5*lato_blocco, -1 + (raggio + lato_blocco)*0.5*CH_C_SQRT_2 - 0.5*lato_blocco));
	blocchetto_condiv->SetRot(Q_from_AngY(-CH_C_PI_4));
	blocchetto_condiv->SetBodyFixed(true);
	mphysicalSystem.Add(blocchetto_condiv);
	*/
	
	



	//arco sinistra shiftato radialmente

	std::shared_ptr<ChBodyEasyBox> blocchetto_s_r[7];

	for (int ix = 0; ix < 6; ++ix) {

		blocchetto_s_r[ix] = std::make_shared<ChBodyEasyBox>(lato_blocco, lato_blocco, lato_blocco,
															1500,      // density
															true,         // ok, contact geometry
															true          // enable visualization geometry
															);

		double offset_azimut = (ix - 1)*(lato_blocco / (raggio + lato_blocco + l_giunto_radiale) + l_giunto_azimut / (raggio + lato_blocco + l_giunto_radiale)) + 0.5*lato_blocco / (raggio + lato_blocco + l_giunto_radiale) + 0.5*l_giunto_azimut / (raggio + lato_blocco + l_giunto_radiale);

		blocchetto_s_r[ix]->SetPos(ChVector<>(-1 - (raggio + 0.5*lato_blocco + lato_blocco + l_giunto_radiale)*sin(offset_azimut), tile_gap_y + 0.5*lato_blocco, -1 + (raggio + 0.5*lato_blocco + lato_blocco + l_giunto_radiale)*cos(offset_azimut)));
		blocchetto_s_r[ix]->SetRot(Q_from_AngY(-offset_azimut));
		blocchetto_s_r[ix]->SetBodyFixed(true);

		mphysicalSystem.Add(blocchetto_s_r[ix]);
	}

	// arco destra

	 std::shared_ptr<ChBodyEasyBox> blocchetto_d[5];

		for (int ix = 0; ix < 4; ++ix) {

			blocchetto_d[ix] = std::make_shared<ChBodyEasyBox>(lato_blocco, lato_blocco, lato_blocco,
												1500,      // density
												true,         // ok, contact geometry
												true          // enable visualization geometry
												);

			double offset_azimut = (ix - 1)*(lato_blocco / raggio + l_giunto_azimut/raggio);

			blocchetto_d[ix]->SetPos(ChVector<>(-1 + (raggio + 0.5*lato_blocco)*sin(offset_azimut), tile_gap_y + 0.5*lato_blocco, -1 + (raggio + 0.5*lato_blocco)*cos(offset_azimut)));
			blocchetto_d[ix]->SetRot(Q_from_AngY(offset_azimut));
			blocchetto_d[ix]->SetBodyFixed(true);

			mphysicalSystem.Add(blocchetto_d[ix]);
		}
	
		//arco destra shiftato radialmente

	std::shared_ptr<ChBodyEasyBox> blocchetto_d_r[7];

	for (int ix = 0; ix < 7; ++ix) {

		blocchetto_d_r[ix] = std::make_shared<ChBodyEasyBox>(lato_blocco, lato_blocco, lato_blocco,
																1500,      // density
																true,         // ok, contact geometry
																true          // enable visualization geometry
																);

		double alpha_blocco = lato_blocco/ (raggio + lato_blocco + l_giunto_radiale);
		double alpha_giunto = l_giunto_azimut / (raggio + lato_blocco + l_giunto_radiale);
		double offset_azimut = (ix - 1)*(alpha_blocco + alpha_giunto) + 0.5*alpha_blocco + 0.5*alpha_giunto;

		blocchetto_d_r[ix]->SetPos(ChVector<>(-1 + (raggio + 0.5*lato_blocco + lato_blocco + l_giunto_radiale)*sin(offset_azimut), tile_gap_y + 0.5*lato_blocco, -1 + (raggio + 0.5*lato_blocco + lato_blocco + l_giunto_radiale)*cos(offset_azimut)));
		blocchetto_d_r[ix]->SetRot(Q_from_AngY(offset_azimut));
		blocchetto_d_r[ix]->SetBodyFixed(true);

		mphysicalSystem.Add(blocchetto_d_r[ix]);
	}
	
// archi contrastanti con blocchi normali (fine)

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
   /* double tile_gap_y = 0.05; // respect to soil*/
    double giunto_gap_x = 0.01;
    double giunto_gap_z = 0.01;

    // Bushing soil-block:
    // (normal direction)
	double stiffness_yy = 12500;  // N/m
	double damping_yy = 100;    // N/ m/s
    double yeld_yy     = 100;  // N
    // (tangential directions/shear)
	double stiffness_yx = 12500;  // N/m
	double stiffness_yz = 12500;  // N/m
	double damping_yx = 100;    // N/ m/s
	double damping_yz = 100;    // N/ m/s
    double yeld_yx     = 100000;  // N
	double yeld_yz = 100000;  // N

    // Bushing block-block:
    // (normal direction)   
	double stiffness_xx_giunto = 10500;  // N/m
	double stiffness_zz_giunto = 10500;  // N/m  
	double damping_xx_giunto = 100;    // N/ m/s
	double damping_zz_giunto = 100;    // N/ m/s
    double yeld_xx_giunto      = 100000;    // N
	double yeld_zz_giunto = 100000;    // N
    // (tangential directions/shear)
	double stiffness_xy_giunto = 3500;  // N/m
	double stiffness_xz_giunto = 3500;  // N/m
	double stiffness_zx_giunto = 3500;  // N/m
	double stiffness_zy_giunto = 3500;  // N/m
	double damping_xy_giunto = 100;    // N/ m/s
	double damping_xz_giunto = 100;    // N/ m/s
	double damping_zx_giunto = 100;    // N/ m/s
	double damping_zy_giunto = 100;    // N/ m/s
	double yeld_xy_giunto = 100000;    // N
	double yeld_xz_giunto = 100000;    // N
	double yeld_zx_giunto = 100000;    // N
	double yeld_zy_giunto = 100000;    // N

    int save_each = 10; // save results each n. of timesteps
	/*
	auto carico = std::make_shared<ChBodyEasyBox>(size_tile_x, size_tile_y, size_tile_z,
		80*tile_density,      // density
		true,         // ok, contact geometry
		true          // enable visualization geometry
		);
	carico->SetPos(ChVector<>(
		1.55,
		2*size_tile_y + tile_gap_y,
		0.62
		));
	mphysicalSystem.Add(carico);
	*/

    int createdbodies = 0;
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
            createdbodies++;
            piastrelle[ix][iz]->SetIdentifier(createdbodies);

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
            
            auto bushing1 = std::make_shared<ChLoadBodyBodyBushingPlasticBreak>(
                                    piastrelle[ix][iz], // body A
                                    floorBody, // body B
                                    ChFrame<>(ChVector<>(-size_tile_x*0.5 + offset_x, tile_gap_y, -size_tile_z*0.5 + offset_z)), //initial frame of bushing in abs space
                                    ChVector<>(stiffness_yx, stiffness_yy, stiffness_yz),    // K stiffness in local frame  [N/m]
                                    ChVector<>(damping_yx, damping_yy, damping_yz),        // R damping in local frame  [N/m/s]
                                    ChVector<>(yeld_yx, yeld_yy, yeld_yz),     // plastic yeld [N]
                                    ChVector<>(0.01, 0.01, 0.01)  // max displ before breaking
                                    );  
            my_loadcontainer->Add(bushing1);

            auto bushing2 = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    floorBody, // body B
                                    ChFrame<>(ChVector<>( size_tile_x*0.5 + offset_x, tile_gap_y, -size_tile_z*0.5 + offset_z)), //initial frame of bushing in abs space
                                    ChVector<>(stiffness_yx, stiffness_yy, stiffness_yz),    // K stiffness in local frame  [N/m]
                                    ChVector<>(damping_yx, damping_yy, damping_yz),        // R damping in local frame  [N/m/s]
                                    ChVector<>(yeld_yx, yeld_yy, yeld_yz)     // plastic yeld [N]
                                    );  
            my_loadcontainer->Add(bushing2);

            auto bushing3 = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    floorBody, // body B
                                    ChFrame<>(ChVector<>( size_tile_x*0.5 + offset_x, tile_gap_y,  size_tile_z*0.5 + offset_z)), //initial frame of bushing in abs space
                                    ChVector<>(stiffness_yx, stiffness_yy, stiffness_yz),    // K stiffness in local frame  [N/m]
                                    ChVector<>(damping_yx, damping_yy, damping_yz),        // R damping in local frame  [N/m/s]
                                    ChVector<>(yeld_yx, yeld_yy, yeld_yz)     // plastic yeld [N]
                                    );  
            my_loadcontainer->Add(bushing3);

            auto bushing4 = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    floorBody, // body B
                                    ChFrame<>(ChVector<>(-size_tile_x*0.5 + offset_x, tile_gap_y,  size_tile_z*0.5 + offset_z)), //initial frame of bushing in abs space
                                    ChVector<>(stiffness_yx, stiffness_yy, stiffness_yz),    // K stiffness in local frame  [N/m]
                                    ChVector<>(damping_yx, damping_yy, damping_yz),        // R damping in local frame  [N/m/s]
                                    ChVector<>(yeld_yx, yeld_yy, yeld_yz)     // plastic yeld [N]
                                    );  
            my_loadcontainer->Add(bushing4);





            // 5- Create the inter-block springs:

            if (ix>0) {

                auto piastrella_prec_x = piastrelle[ix-1][iz];

                auto bushing_1h = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    piastrella_prec_x,  // body B
									ChFrame<>(ChVector<>(offset_x - size_tile_x*0.5, tile_gap_y, offset_z + size_tile_z*0.5)), //initial frame of bushing in abs space
									ChVector<>(stiffness_xx_giunto, stiffness_xy_giunto, stiffness_xz_giunto),    // K stiffness in local frame  [N/m]
                                    ChVector<>(damping_xx_giunto, damping_xy_giunto, damping_xz_giunto),        // R damping in local frame  [N/m/s]
                                    ChVector<>(yeld_xx_giunto, yeld_xy_giunto, yeld_xz_giunto)     // plastic yeld [N]
                                    );  
                my_loadcontainer->Add(bushing_1h);

                auto bushing_2h = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    piastrella_prec_x,  // body B
									ChFrame<>(ChVector<>(offset_x - size_tile_x*0.5, tile_gap_y, offset_z - size_tile_z*0.5)), //initial frame of bushing in abs space
									ChVector<>(stiffness_xx_giunto, stiffness_xy_giunto, stiffness_xz_giunto),    // K stiffness in local frame  [N/m]
                                    ChVector<>(damping_xx_giunto, damping_xy_giunto, damping_xz_giunto),        // R damping in local frame  [N/m/s]
                                    ChVector<>(yeld_xx_giunto, yeld_xy_giunto, yeld_xz_giunto)     // plastic yeld [N]
                                    );  
                my_loadcontainer->Add(bushing_2h);
            }

            if (iz>0) {

                auto piastrella_prec_z = piastrelle[ix][iz-1];

                auto bushing_1h = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    piastrella_prec_z,  // body B
									ChFrame<>(ChVector<>(offset_x - size_tile_x*0.5, tile_gap_y, offset_z - size_tile_z*0.5)), //initial frame of bushing in abs space
									ChVector<>(stiffness_zx_giunto, stiffness_zy_giunto, stiffness_zz_giunto),    // K stiffness in local frame  [N/m]
                                    ChVector<>(damping_zx_giunto, damping_zy_giunto, damping_zz_giunto),        // R damping in local frame  [N/m/s]
                                    ChVector<>(yeld_zx_giunto, yeld_zy_giunto, yeld_zz_giunto)     // plastic yeld [N]
                                    );  
                my_loadcontainer->Add(bushing_1h);

                auto bushing_2h = std::make_shared<ChLoadBodyBodyBushingPlastic>(
                                    piastrelle[ix][iz], // body A
                                    piastrella_prec_z,  // body B
									ChFrame<>(ChVector<>(offset_x + size_tile_x*0.5, tile_gap_y, offset_z - size_tile_z*0.5)), //initial frame of bushing in abs space
									ChVector<>(stiffness_zx_giunto, stiffness_zy_giunto, stiffness_zz_giunto),    // K stiffness in local frame  [N/m]
                                    ChVector<>(damping_zx_giunto, damping_zy_giunto, damping_zz_giunto),        // R damping in local frame  [N/m/s]
                                    ChVector<>(yeld_zx_giunto, yeld_zy_giunto, yeld_zz_giunto)     // plastic yeld [N]
                                    );  
                my_loadcontainer->Add(bushing_2h); 

            }
        }
    }

    // 6- Create a rigid wheel 
	
    auto wheelBody1 = std::make_shared<ChBodyEasyCylinder>(0.4, 0.285,  // R, h
                                                        600,         // density
                                                        true,        //  contact geometry
                                                        true          // enable visualization geometry
                                                        );
    wheelBody1->SetPos(ChVector<>(0, 0.60, 0));
    wheelBody1->SetRot(Q_from_AngX(CH_C_PI_2)); // 90 deg
	wheelBody1->SetPos_dt(ChVector<>(3, 0, 0));

	mphysicalSystem.Add(wheelBody1);

	auto wheelBody2 = std::make_shared<ChBodyEasyCylinder>(0.1, 0.285,  // R, h
		800,         // density
		true,        //  contact geometry
		true          // enable visualization geometry
		);
	wheelBody2->SetPos(ChVector<>(2.55, 0.50, 0.1));
	wheelBody2->SetRot(Q_from_AngX(CH_C_PI_2)); // 90 deg
	wheelBody2->SetPos_dt(ChVector<>(-1, 0, 0));


    mphysicalSystem.Add(wheelBody2);
	

	/*
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

/*
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
				*/
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

/*
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
	
	*/
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

/*
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

	
	
	*/
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

    // put all results into a subdirectory
    ChFileutils::MakeDirectory("results");

    //
    // THE SOFT-REAL-TIME CYCLE
    //

    while (application.GetDevice()->run()) {
        application.BeginScene();

        application.DrawAll();

        // This performs the integration timestep!
        application.DoStep();


        // Do some output to disk, for later postprocessing
        if (save_each && (mphysicalSystem.GetStepcount() % save_each  == 0))
        {
            // a) Use the contact callback object to save contacts:
            char contactfilename[200];
            sprintf(contactfilename, "%s%05d%s", "results/contacts", mphysicalSystem.GetStepcount(), ".txt");  // ex: contacts00020.tx
            _contact_reporter_class my_contact_rep;
            ChStreamOutAsciiFile result_contacts(contactfilename);
            my_contact_rep.mfile = &result_contacts;
            mphysicalSystem.GetContactContainer()->ReportAllContacts(&my_contact_rep);

            // b) Save rigid body positions and rotations
            char bodyfilename[200];
            sprintf(bodyfilename, "%s%05d%s", "results/bodies", mphysicalSystem.GetStepcount(), ".txt");  // ex: bodies00020.tx
            ChStreamOutAsciiFile result_bodies(bodyfilename);
            ChSystem::IteratorBodies mbodies = mphysicalSystem.IterBeginBodies();
            while (mbodies != mphysicalSystem.IterEndBodies()) {
                result_bodies   << (*mbodies)->GetIdentifier()  << ", " 
                                << (*mbodies)->GetPos().x()  << ", "
                                << (*mbodies)->GetPos().y()  << ", "
                                << (*mbodies)->GetPos().z()  << ", "
                                << (*mbodies)->GetRot().e0()  << ", "
                                << (*mbodies)->GetRot().e1()  << ", "
                                << (*mbodies)->GetRot().e2()  << ", "
                                << (*mbodies)->GetRot().e3()  << "\n";
                ++mbodies;
            }

            
            // b) Save spring reactions
            char bushingfilename[200];
            sprintf(bushingfilename, "%s%05d%s", "results/bushings", mphysicalSystem.GetStepcount(), ".txt");  // ex: springs00020.tx
            ChStreamOutAsciiFile result_bushings(bushingfilename);
            auto mitem = mphysicalSystem.IterBeginOtherPhysicsItems();
            while (mitem != mphysicalSystem.IterEndOtherPhysicsItems()) {
                if (auto mloadcontainer = std::dynamic_pointer_cast<ChLoadContainer>((*mitem))) {
                     for (auto mload : mloadcontainer->GetLoadList())  {
                         if (auto mbushing = std::dynamic_pointer_cast<ChLoadBodyBodyBushingPlastic>((mload))) {
                            result_bushings << mbushing->GetBodyA()->GetIdentifier()  << ", "
                                            << mbushing->GetBodyB()->GetIdentifier()  << ", "
                                            << mbushing->GetAbsoluteFrameA().GetPos().x()  << ", " 
                                            << mbushing->GetAbsoluteFrameA().GetPos().y()  << ", " 
                                            << mbushing->GetAbsoluteFrameA().GetPos().z()  << ", " 
                                            << mbushing->GetBushingForce().x() << ", " 
                                            << mbushing->GetBushingForce().y() << ", " 
                                            << mbushing->GetBushingForce().z() << "\n"; 
                         }
                     }
                }
                ++mitem;
            }
            
        }



        application.EndScene();
    }

    return 0;
}
