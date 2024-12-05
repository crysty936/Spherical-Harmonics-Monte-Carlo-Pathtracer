#include "TestGameMode.h"
#include "Camera/Camera.h"
#include "Scene/SceneManager.h"
#include "Scene/Scene.h"
#include "Renderer/Drawable/ShapesUtils/BasicShapes.h"
#include "Controller/ControllerBase.h"
#include "glm/common.hpp"
#include <stdlib.h>
#include "Renderer/Model/3D/Assimp/AssimpModel3D.h"
#include "Core/SceneHelper.h"
#include "Renderer/RHI/Resources/MeshDataContainer.h"
#include "Renderer/ForwardRenderer.h"
#include "Renderer/RHI/Resources/VertexInputLayout.h"
#include "Renderer/Drawable/ShapesUtils/BasicShapesData.h"
#include "Renderer/RHI/RHI.h"
#include "Renderer/Material/MaterialsManager.h"
#include "Utils/InlineVector.h"
#include "Renderer/Material/EngineMaterials/RenderMaterial_Parallax.h"
#include "Renderer/Material/EngineMaterials/RenderMaterial_Billboard.h"
#include "Renderer/DrawDebugHelpers.h"
#include "Renderer/Model/3D/Assimp/AssimpModel3DPBRSphere.h"
#include "imgui.h"
#include "Renderer/Model/3D/Assimp/AssimpModel3DSphereHarmonicsDebug.h"

TestGameMode GGameMode = {};

TestGameMode::TestGameMode() = default;
TestGameMode::~TestGameMode() = default;

void TestGameMode::Init()
{
	// Scene setup 
	SceneManager& sManager = SceneManager::Get();
	Scene& currentScene = sManager.GetCurrentScene();
	GameCamera = currentScene.GetCurrentCamera();

	// Push camera back a bit
	if (TransformObjPtr parentShared = GameCamera->GetParent().lock())
	{
		// Move the camera parent
		parentShared->SetRelativeLocation(glm::vec3(4.2f, 0.38f, 4.8f));
		parentShared->SetRotationDegrees(glm::vec3(0.f, 42.f, 0.f));
	}

	{
	 	DirLight = SceneHelper::CreateVisualEntity<LightSource>("Directional Light");
	 	DirLight->LData.Type = ELightType::Directional;
	 	DirLight->SetRelativeLocation({ -2.0f, 20.0f, -1.0f });
	 	DirLight->SetRotationDegrees(glm::vec3(80.f, 0.f, 0.f));
	}

	constexpr float linear = 0.0014f;
	constexpr float quadratic = 0.000007f;

	MainModel = SceneHelper::CreateVisualEntity<AssimpModel3D>("../Data/Models/low_poly_suzanne/Monkey.obj", "Monke", glm::vec3(1.f, 1.f, 1.f));
}

struct Sphere
{
	glm::vec3 Origin = glm::vec3(0.f, 0.f, 0.f);
	float Radius = 0.f;
	glm::vec3 Color = glm::vec3(0.f, 0.f, 0.f);
	bool bIsEmissive = false;
};


static eastl::vector<Sphere> spheres = {
	{glm::vec3(0.f, 0.f, 0.f), 5.f, glm::vec3(1.f, 0.f, 0.f), false},
	//{glm::vec3(15.f, 0.f, 0.f), 5.f, glm::vec3(0.0f, 0.2f, 0.5f), true},
	//{glm::vec3(0.f, -50.f, 0.f), 45.f, glm::vec3(0.f, 1.f, 0.f), false}
};

void TestGameMode::Tick(float inDeltaT)
{
	ImGui::Begin("Test Game Settings");

	glm::vec3 forward = glm::vec3(0.f, 0.f, 1.f);
	glm::vec3 right = glm::vec3(1.f, 0.f, 0.f);
	glm::vec3 up = glm::vec3(0.f, 1.f, 0.f);

	const eastl::shared_ptr<TransformObject>& usedObj = MainModel;

	if (usedObj)
	{
 		const glm::quat& rot = usedObj->GetAbsoluteTransform().Rotation;
 
 		// order of operations matters, considered like a matrix
 		forward = glm::normalize(rot * forward);
 		right = glm::normalize(rot * right);
 		up = glm::normalize(rot * up);
 
 		const glm::vec3 start = usedObj->GetAbsoluteTransform().Translation;
 
 		DrawDebugHelpers::DrawDebugLine(start, start + forward * 2.f, glm::vec3(0.f, 0.f, 1.f));
 		DrawDebugHelpers::DrawDebugLine(start, start + right * 2.f, glm::vec3(1.f, 0.f, 0.f));
 		DrawDebugHelpers::DrawDebugLine(start, start + up * 2.f, glm::vec3(0.f, 1.f, 0.f));
	}


	ImGui::End();

}
