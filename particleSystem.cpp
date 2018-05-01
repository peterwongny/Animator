#pragma warning(disable : 4786)

#include "particleSystem.h"


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <math.h>
#include <limits.h>


/***************
 * Constructors
 ***************/

ParticleSystem::ParticleSystem()
{
	forces.push_back(Vec3d(0, -9.8, 0)); //gravity

	num_particles = 0;
	max_particles = 50;
	bake_fps = 30;
	bake_start_time = -1;
	bake_end_time = -1;

	simulate = false;
	dirty = false;


}





/*************
 * Destructor
 *************/

ParticleSystem::~ParticleSystem() 
{
	clearBaked();

}


/******************
 * Simulation fxns
 ******************/

/** Start the simulation */
void ParticleSystem::startSimulation(float t)
{
    
	clearBaked();
	bake_start_time = t;

	// These values are used by the UI ...
	// -ve bake_end_time indicates that simulation
	// is still progressing, and allows the
	// indicator window above the time slider
	// to correctly show the "baked" region
	// in grey.
	bake_end_time = -1;
	simulate = true;
	dirty = true;

}

/** Stop the simulation */
void ParticleSystem::stopSimulation(float t)
{
	num_particles = 0;
	bake_end_time = t;

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Reset the simulation */
void ParticleSystem::resetSimulation(float t)
{
    
	bake_start_time = -1;
	bake_end_time = -1;

	// These values are used by the UI
	simulate = false;
	dirty = true;

}

/** Compute forces and update particles **/
void ParticleSystem::computeForcesAndUpdateParticles(float t)
{
	
	if (simulate) { // do compute

		double dt = 1.0 / bake_fps;


		bool add_particle = false;

		int num_frame = round((t - bake_start_time) * bake_fps);

		if (num_frame % 2 == 0) {

			add_particle = true;

			if (num_particles < max_particles) {
				num_particles += 1;

			}
		}

			vector<Particle> old_particles = particles;
			particles.clear();


			for (int i = 0; i != num_particles; i++) {
				if (add_particle && i == 0) {

					Particle new_particle;

					float r = rand() % 100 + 300;
					float m = r / 1000.0;

					// initial postition
					Vec4d world_corr;
					
					double rx = (rand() % 300) / 1000.0;
					double ry = (rand() % 300) / 1000.0;
					double rz = (rand() % 300) / 1000.0;

					world_corr = get_world_cord(rx, ry, rz);

					new_particle.x[0] = world_corr[0];
					new_particle.x[1] = world_corr[1];
					new_particle.x[2] = world_corr[2];


					// initial velocity
					new_particle.v[0] = rx;
					new_particle.v[1] = ry;
					new_particle.v[2] = rz;

					particles.push_back(new_particle);

				}
				else {

					int old_i = i;
					if (add_particle) old_i = i - 1;

					Particle p = old_particles[old_i];

					p.x += p.v * dt;

					p.v += p.f *dt / p.m;

					//force
					p.f = Vec3d(0.0, -9.8, 0.0) * p.m;

					particles.push_back(p);

	
					if (particles.size() > max_particles) particles.pop_back();

				}

			}

		}

		bakeParticles(t);




}



Vec4d ParticleSystem::get_world_cord(double x, double y, double z) {

	Vec4d localCoord(x, y, z, 1.0);

	return trans_matrix*localCoord;
}



/** Render particles */
void ParticleSystem::drawParticles(float t)
{
	//printf("%d,%d,", num_particles, particles.size());

	auto iter = baked_frames.find(get_frame(t));
	if (iter == baked_frames.end()) return;


	vector<Particle>& baked_particles = iter->second;
	for (Particle& particle : baked_particles) {
		glPushMatrix();
		setDiffuseColor(1.0,0.0,0.0);
		glTranslated(particle.x[0], particle.x[1], particle.x[2]);
		drawSphere(0.1);
		glPopMatrix();
	}
	// TODO
}





/** Adds the current configuration of particles to
  * your data structure for storing baked particles **/
void ParticleSystem::bakeParticles(float t) 
{
	auto it = baked_frames.find(get_frame(t));
	if (it == baked_frames.end()) { //this frame not yet baked
		baked_frames.insert(std::make_pair(get_frame(t), particles));
	}
	else {
		it->second = particles;
	}
}

/** Clears out your data structure of baked particles */
void ParticleSystem::clearBaked()
{
	baked_frames.clear();
	// TODO
}



//Particle ParticleSystem::newParticle() {
//
//}

