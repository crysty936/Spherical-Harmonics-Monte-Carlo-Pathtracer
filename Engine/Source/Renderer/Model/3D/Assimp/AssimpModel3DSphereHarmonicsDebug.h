#pragma once
#include "Renderer/Model/3D/Model3D.h"
#include "EASTL/string.h"
#include "Core/EngineUtils.h"
#include "assimp/material.h"
#include "Renderer/RenderCommand.h"
#include "AssimpModel3D.h"

class AssimpModel3DSphereHarmonicsDebug : public AssimpModel3D
{
public:
	AssimpModel3DSphereHarmonicsDebug(const eastl::string& inPath, const eastl::string& inName);
	virtual ~AssimpModel3DSphereHarmonicsDebug();

protected:
	eastl::shared_ptr<RHIShader> CreateShaders(const class VertexInputLayout& inLayout) const override;
	eastl::shared_ptr<RenderMaterial> CreateMaterial(const struct aiMesh& inMesh, bool& outMatExists) const override;

};