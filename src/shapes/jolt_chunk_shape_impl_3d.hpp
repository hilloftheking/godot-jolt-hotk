#pragma once

#include "shapes/jolt_shape_impl_3d.hpp"
#include "shapes/jolt_custom_shape_type.hpp"

class JoltCustomChunkShape final : public JPH::Shape {
public:
	static void register_type();

	JoltCustomChunkShape()
		: JPH::Shape(JoltCustomShapeType::CHUNK, JoltCustomShapeSubType::CHUNK) { }

	virtual JPH::Vec3 GetCenterOfMass() const override { return {0, 0, 0}; }

	virtual JPH::AABox GetLocalBounds() const override {
		return JPH::AABox(JPH::Vec3(0.0f, 0.0f, 0.0f), JPH::Vec3((float)size, (float)size, (float)size));
	}

	virtual JPH::uint GetSubShapeIDBitsRecursive() const override { return 1; }

	virtual float GetInnerRadius() const override { return size * 0.5f; }

	virtual JPH::MassProperties GetMassProperties() const override {
		JPH::MassProperties mass_properties;

		// I don't think this matters
		mass_properties.mMass = 1.0f;
		mass_properties.mInertia = JPH::Mat44::sScale(0.1f);

		return mass_properties;
	}

	virtual const JPH::PhysicsMaterial* GetMaterial([[maybe_unused]] const JPH::SubShapeID& inSubShapeID) const override {
		return material;
	}

	virtual JPH::Vec3 GetSurfaceNormal(
		[[maybe_unused]] const JPH::SubShapeID& inSubShapeID,
		[[maybe_unused]] JPH::Vec3Arg inLocalSurfacePosition
	) const override {
		return last_ray_normal;
	}

	virtual void GetSubmergedVolume(
		[[maybe_unused]] JPH::Mat44Arg inCenterOfMassTransform,
		[[maybe_unused]] JPH::Vec3Arg inScale,
		[[maybe_unused]] const JPH::Plane& inSurface,
		[[maybe_unused]] float& outTotalVolume,
		[[maybe_unused]] float& outSubmergedVolume,
		[[maybe_unused]] JPH::Vec3& outCenterOfBuoyancy
		#ifdef JPH_DEBUG_RENDERER
		, [[maybe_unused]] JPH::RVec3Arg inBaseOffset
		#endif
	) const override {
		WARN_PRINT_ONCE("GetSubmergedVolume");
	}

	#ifdef JPH_DEBUG_RENDERER
	virtual void Draw(
		[[maybe_unused]] JPH::DebugRenderer* inRenderer,
		[[maybe_unused]] JPH::RMat44Arg inCenterOfMassTransform,
		[[maybe_unused]] JPH::Vec3Arg inScale,
		[[maybe_unused]] JPH::ColorArg inColor,
		[[maybe_unused]] bool inUseMaterialColors,
		[[maybe_unused]] bool inDrawWireframe
	) const override { }
	#endif

	virtual bool CastRay(
		const JPH::RayCast& inRay,
		const JPH::SubShapeIDCreator& inSubShapeIDCreator,
		JPH::RayCastResult& ioHit
	) const override;

	virtual void CastRay(
		const JPH::RayCast& inRay,
		[[maybe_unused]] const JPH::RayCastSettings& inRayCastSettings,
		const JPH::SubShapeIDCreator& inSubShapeIDCreator,
		JPH::CastRayCollector& ioCollector,
		[[maybe_unused]] const JPH::ShapeFilter& inShapeFilter = {}
	) const override {
		// Just do one hit. I don't think it should matter
		JPH::RayCastResult rayHit;
		rayHit.mBodyID = JPH::TransformedShape::sGetBodyID(ioCollector.GetContext());
		if (CastRay(inRay, inSubShapeIDCreator, rayHit)) {
			ioCollector.AddHit(rayHit);
		}
	}

	virtual void CollidePoint(
		[[maybe_unused]] JPH::Vec3Arg inPoint,
		[[maybe_unused]] const JPH::SubShapeIDCreator& inSubShapeIDCreator,
		[[maybe_unused]] JPH::CollidePointCollector& ioCollector,
		[[maybe_unused]] const JPH::ShapeFilter& inShapeFilter = {}
	) const override {
		WARN_PRINT_ONCE("CollidePoint");
	}

	virtual void CollideSoftBodyVertices(
		[[maybe_unused]] JPH::Mat44Arg inCenterOfMassTransform,
		[[maybe_unused]] JPH::Vec3Arg inScale,
		[[maybe_unused]] JPH::SoftBodyVertex* ioVertices,
		[[maybe_unused]] JPH::uint inNumVertices,
		[[maybe_unused]] float inDeltaTime,
		[[maybe_unused]] JPH::Vec3Arg inDisplacementDueToGravity,
		[[maybe_unused]] int inCollidingShapeIndex
	) const override { }

	virtual void GetTrianglesStart(
		[[maybe_unused]] GetTrianglesContext& ioContext,
		[[maybe_unused]] const JPH::AABox& inBox,
		[[maybe_unused]] JPH::Vec3Arg inPositionCOM,
		[[maybe_unused]] JPH::QuatArg inRotation,
		[[maybe_unused]] JPH::Vec3Arg inScale
	) const override {
		WARN_PRINT_ONCE("GetTrianglesStart");
	}

	virtual int GetTrianglesNext(
		[[maybe_unused]] GetTrianglesContext& ioContext,
		[[maybe_unused]] int inMaxTrianglesRequested,
		[[maybe_unused]] JPH::Float3* outTriangleVertices,
		[[maybe_unused]] const JPH::PhysicsMaterial** outMaterials = nullptr
	) const override {
		WARN_PRINT_ONCE("GetTrianglesNext");
		return 0;
	}

	virtual Stats GetStats() const override { return Stats{0, 0}; }

	virtual float GetVolume() const override { return 1.0; }

	JPH::RefConst<JPH::PhysicsMaterial> material;

	int size = 0;

	const uint8_t* blocks = nullptr;

	// Store raycast normals since they are all that is needed from GetSurfaceNormal
	mutable JPH::Vec3 last_ray_normal;
};

class JoltChunkShapeImpl3D final : public JoltShapeImpl3D {
public:
	ShapeType get_type() const override { return ShapeType::SHAPE_CUSTOM; }

	bool is_convex() const override { return false; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	float get_margin() const override { return 0.0f; }

	void set_margin([[maybe_unused]] float p_margin) override { }

private:
	JPH::ShapeRefC _build() const override {
		auto* chunk_shape = new JoltCustomChunkShape();
		chunk_shape->size = size;
		chunk_shape->blocks = blocks.ptr();
		return chunk_shape;
	}

	int size = 0;

	PackedByteArray blocks;
};
