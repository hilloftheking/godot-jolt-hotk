#include "shapes/jolt_chunk_shape_impl_3d.hpp"

#include "Jolt/Geometry/RayAABox.h"

namespace {

JPH::Shape* construct_chunk() {
	return new JoltCustomChunkShape();
}

void collide_shape_vs_chunk(
	const JPH::Shape* p_shape1,
	const JPH::Shape* p_shape2,
	[[maybe_unused]] JPH::Vec3Arg p_scale1,
	JPH::Vec3Arg p_scale2,
	JPH::Mat44Arg p_center_of_mass_transform1,
	JPH::Mat44Arg p_center_of_mass_transform2,
	const JPH::SubShapeIDCreator& p_sub_shape_id_creator1,
	const JPH::SubShapeIDCreator& p_sub_shape_id_creator2,
	const JPH::CollideShapeSettings& p_collide_shape_settings,
	JPH::CollideShapeCollector& p_collector,
	const JPH::ShapeFilter& p_shape_filter
) {
	ERR_FAIL_COND(p_shape2->GetSubType() != JoltCustomShapeSubType::CHUNK);

	const auto* chunk = static_cast<const JoltCustomChunkShape*>(p_shape2);
	if (!chunk->blocks || !chunk->size) {
		// Chunk has not been configured yet
		return;
	}

	// Get AABB in world space first. For some reason JoltCustomMotionShape::GetWorldSpaceBounds is unimplemented.
	JPH::AABox local_aabb = p_shape1->GetLocalBounds().Scaled(p_scale1).Transformed(
		p_center_of_mass_transform1
	);
	// Now translate it to be local to the chunk
	local_aabb.Translate(-p_center_of_mass_transform2.GetTranslation());

	// Each block in [start, end) is checked
	Vector3i start;
	Vector3i end;
	for (int i = 0; i < 3; ++i) {
		start[i] = (int)local_aabb.mMin[(JPH::uint)i];
		start[i] = Math::clamp<int>(start[i], 0, chunk->size - 1);
		// Add one so that it gets rounded up
		end[i] = (int)(local_aabb.mMax[(JPH::uint)i] + 1.0f);
		end[i] = Math::clamp<int>(end[i], 0, chunk->size);
	}

	const JPH::Vec3 half(0.5f, 0.5f, 0.5f);
	JPH::BoxShape box_shape(half, 0.0f, chunk->material);

	for (int x = start.x; x < end.x; ++x) {
		for (int y = start.y; y < end.y; ++y) {
			for (int z = start.z; z < end.z; ++z) {
				// If block at x,y,z in 3D array blocks is 0 then don't test collision
				int block_ind = x * chunk->size * chunk->size + y * chunk->size + z;
				if (!chunk->blocks[block_ind]) {
					continue;
				}

				// Add half to box position because the position will be for the center of the box
				JPH::Vec3 box_pos = JPH::Vec3((float)x, (float)y, (float)z) + half;
				JPH::Mat44 box_trans = p_center_of_mass_transform2.PostTranslated(box_pos);

				// Call built in collision function for BoxShape
				JPH::CollisionDispatch::sCollideShapeVsShape(
					p_shape1,
					&box_shape,
					p_scale1,
					JPH::Vec3(1, 1, 1),
					p_center_of_mass_transform1,
					box_trans,
					p_sub_shape_id_creator1,
					p_sub_shape_id_creator2,
					p_collide_shape_settings,
					p_collector,
					p_shape_filter
				);

				if (p_collector.ShouldEarlyOut()) {
					return;
				}
			}
		}
	}
}

} // namespace

void JoltCustomChunkShape::register_type() {
	JPH::ShapeFunctions& shape_functions = JPH::ShapeFunctions::sGet(JoltCustomShapeSubType::CHUNK);

	shape_functions.mConstruct = construct_chunk;
	shape_functions.mColor = JPH::Color::sDarkRed;

	for (const JPH::EShapeSubType s : JPH::sConvexSubShapeTypes) {
		JPH::CollisionDispatch::sRegisterCollideShape(
			s,
			JoltCustomShapeSubType::CHUNK,
			collide_shape_vs_chunk
		);

		JPH::CollisionDispatch::sRegisterCollideShape(
			JoltCustomShapeSubType::CHUNK,
			s,
			JPH::CollisionDispatch::sReversedCollideShape
		);
	}
}

bool JoltCustomChunkShape::CastRay(
	const JPH::RayCast& inRay,
	const JPH::SubShapeIDCreator& inSubShapeIDCreator,
	JPH::RayCastResult& ioHit
) const {
	if (!blocks || !size) {
		// Chunk hasn't been configured yet
		return false;
	}

	// Fast Voxel Traversal
	Vector3 d = to_godot(inRay.mDirection);
	Vector3 dnorm = d.normalized();
	real_t t = 0.0;
	Vector3i ind;
	Vector3i step;
	Vector3 delta;
	Vector3 dist;
	Vector3 tmax;

	// Figure out how much to step on each axis
	for (int i = 0; i < 3; i++) {
		ind[i] = (int)Math::floor(inRay.mOrigin[(JPH::uint)i]);
		if (d[i] >= 0.0f) {
			step[i] = 1;
			dist[i] = (real_t)ind[i] + 1.0f - inRay.mOrigin[(JPH::uint)i];
		} else {
			step[i] = -1;
			dist[i] = inRay.mOrigin[(JPH::uint)i] - (real_t)ind[i];
		}

		if (d[i]) {
			delta[i] = Math::abs(1.0f / dnorm[i]);
			tmax[i] = delta[i] * dist[i];
		} else {
			delta[i] = FLT_MAX;
			tmax[i] = FLT_MAX;
		}
	}

	// Used for extents of box
	static const JPH::Vec3 half(0.5f, 0.5f, 0.5f);

	real_t len = d.length();
	while (t <= len) {
		bool inside = true;
		for (int i = 0; i < 3; i++) {
			if (ind[i] < 0 || ind[i] >= size) {
				inside = false;
				break;
			}
		}
		if (inside) {
			if (blocks[ind.x * size * size + ind.y * size + ind.z] != 0) {
				JPH::Vec3 box_pos = JPH::Vec3(ind.x + 0.5f, ind.y + 0.5f, ind.z + 0.5f);
				float fraction = max(JPH::RayAABox(inRay.mOrigin, JPH::RayInvDirection(inRay.mDirection), box_pos - half, box_pos + half), 0.0f);
				if (fraction < ioHit.mFraction) {
					ioHit.mFraction = fraction;
					ioHit.mSubShapeID2 = inSubShapeIDCreator.GetID();

					// Go ahead and store the normal of the hit since it will be retrieved from GetSurfaceNormal
					JPH::Vec3 hit_pos = inRay.mOrigin + ioHit.mFraction * inRay.mDirection;
					hit_pos -= box_pos;
					JPH::uint lowest_comp_index = (JPH::uint)((hit_pos.Abs() - half).Abs().GetLowestComponentIndex());

					// Calculate normal
					JPH::Vec3 normal = JPH::Vec3::sZero();
					normal.SetComponent(lowest_comp_index, hit_pos[(JPH::uint)lowest_comp_index] > 0.0f ? 1.0f : -1.0f);
					last_ray_normal = normal;

					return true;
				}
			}
		}

		// Figure out which direction to go
		int i = 0;
		if (tmax.x < tmax.y) {
			if (tmax.x < tmax.z) {
				i = 0;
			} else {
				i = 2;
			}
		} else {
			if (tmax.y < tmax.z) {
				i = 1;
			} else {
				i = 2;
			}
		}

		ind[i] += step[i];
		t = tmax[i];
		tmax[i] += delta[i];
	}

	return false;
}

Variant JoltChunkShapeImpl3D::get_data() const {
	Dictionary d;
	d["size"] = size;
	d["blocks"] = blocks;
	return d;
}

void JoltChunkShapeImpl3D::set_data(const Variant& p_data) {
	ON_SCOPE_EXIT {
		_invalidated();
	};

	destroy();

	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);
	const Dictionary& d = p_data;

	const Variant maybe_size = d.get("size", {});
	ERR_FAIL_COND(maybe_size.get_type() != Variant::INT);

	const Variant maybe_blocks = d.get("blocks", {});
	ERR_FAIL_COND(maybe_blocks.get_type() != Variant::PACKED_BYTE_ARRAY);

	size = maybe_size;
	blocks = maybe_blocks;
}
