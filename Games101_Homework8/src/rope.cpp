#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL
{
	Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
	{
		// Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

		for (int i = 0; i < num_nodes; i++)
		{
			Vector2D pos = start + (end - start) * ((double)i / ((double)num_nodes - 1));
			masses.push_back(new Mass(pos, node_mass, false));
		}

		for (int i = 0; i < num_nodes - 1; i++)
		{
			springs.push_back(new Spring(masses[i], masses[i + 1], k));
		}

		for (auto& i : pinned_nodes)
		{
			masses[i]->pinned = true;
		}
	}

	void Rope::simulateEuler(float delta_t, Vector2D gravity)
	{
		for (auto& s : springs)
		{
			// Use Hooke's law to calculate the force on a node
			auto len = (s->m2->position - s->m1->position);
			Vector2D f = s->k * (len.unit()) * (len.norm() - s->rest_length);
			s->m1->forces += f;
			s->m2->forces -= f;
		}

		for (auto& m : masses)
		{
			if (!m->pinned)
			{
				// Add the force due to gravity, then compute the new velocity and position
				m->forces += gravity * m->mass;
				// Add global damping
				float dp = 0.005f;
				m->forces += -dp * m->velocity;
				auto a = m->forces / m->mass;
				// 显式欧拉方法
				/*m->position += m->velocity * delta_t;
				m->velocity += a * delta_t;*/
				// 隐式欧拉方法
				m->velocity += a * delta_t;
				m->position += m->velocity * delta_t;
			}
			// Reset all forces on each mass
			m->forces = Vector2D(0, 0);
		}
	}

	void Rope::simulateVerlet(float delta_t, Vector2D gravity)
	{
		for (auto& s : springs)
		{
			// Simulate one timestep of the rope using explicit Verlet （solving constraints)
			auto len = (s->m2->position - s->m1->position);
			Vector2D f = s->k * (len.unit()) * (len.norm() - s->rest_length);
			s->m1->forces += f;
			s->m2->forces -= f;
		}

		for (auto& m : masses)
		{
			if (!m->pinned)
			{
				Vector2D temPos = m->position;
				m->forces += gravity * m->mass;
				auto a = m->forces / m->mass;
				// Set the new position of the rope mass
				// Add global Verlet damping
				float dp = 0.0001f;
				// 显式Verlet方法
				m->position = temPos + (1 - dp) * (temPos - m->last_position) + a * delta_t * delta_t;
				m->last_position = temPos;
			}
			m->forces = Vector2D(0, 0);
		}
	}
}
