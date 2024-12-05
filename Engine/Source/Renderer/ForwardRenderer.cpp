#include <assert.h>
#include "Renderer/ForwardRenderer.h"
#include "Core/EngineUtils.h"
#include "Core/EngineCore.h"
#include "Scene/Scene.h"
#include "Camera/Camera.h"
#include "Scene/SceneManager.h"
#include "glm/ext/matrix_float4x4.hpp"
#include "glm/ext/matrix_clip_space.hpp"
#include "glm/trigonometric.hpp"
#include "Entity/Entity.h"
#include "Renderer/Drawable/Drawable.h"
#include "EASTL/shared_ptr.h"
#include "Renderer/Material/RenderMaterial.h"
#include "Renderer/RHI/Resources/MeshDataContainer.h"
#include "Renderer/Material/MaterialsManager.h"
#include "Core/SceneHelper.h"
#include "Renderer/Material/EngineMaterials/DepthMaterial.h"
#include "Core/WindowsPlatform.h"
#include "glm/gtc/integer.hpp"

#include "InputSystem/InputType.h"
#include "Window/WindowsWindow.h"
#include "InputSystem/InputSystem.h"

#include "Renderer/RHI/RHI.h"
#include "Renderer/RHI/Resources/RHIShader.h"
#include "Renderer/RHI/Resources/RHITexture.h"
#include "Drawable/ShapesUtils/BasicShapes.h"
#include "EASTL/stack.h"
#include "Material/EngineMaterials/RenderMaterial_Debug.h"
#include "DrawDebugHelpers.h"
#include "RenderUtils.h"
#include "Math/AABB.h"
#include "imgui.h"
#include "ShaderTypes.h"
#include "Math/SphericalHarmonics.h"
#include "Math/MathUtils.h"
#include "Math/SphericalHarmonicsRotation.h"

eastl::shared_ptr<RHIFrameBuffer> GlobalFrameBuffer = nullptr;
eastl::shared_ptr<RHITexture2D> GlobalRenderTexture = nullptr;

eastl::shared_ptr<RHIFrameBuffer> AuxiliaryFrameBuffer = nullptr;
eastl::shared_ptr<RHITexture2D> AuxiliaryRenderTexture = nullptr;

ForwardRenderer::ForwardRenderer(const WindowProperties& inMainWindowProperties)
	: Renderer(inMainWindowProperties)
{

}

ForwardRenderer::~ForwardRenderer() = default;

void ForwardRenderer::InitInternal()
{
	GlobalFrameBuffer = RHI::Get()->CreateDepthStencilFrameBuffer();

	const WindowsWindow& currentWindow = GEngine->GetMainWindow();
	const WindowProperties& props = currentWindow.GetProperties();

	// HDR Texture
	GlobalRenderTexture = RHI::Get()->CreateRenderTexture(props.Width, props.Height, ERHITexturePrecision::Float16, ERHITextureFilter::Linear);
	RHI::Get()->AttachTextureToFramebufferColor(*GlobalFrameBuffer, GlobalRenderTexture);

	AuxiliaryFrameBuffer = RHI::Get()->CreateDepthStencilFrameBuffer();
	AuxiliaryRenderTexture = RHI::Get()->CreateRenderTexture(props.Width, props.Height, ERHITexturePrecision::Float16, ERHITextureFilter::Linear);
	RHI::Get()->AttachTextureToFramebufferColor(*AuxiliaryFrameBuffer, AuxiliaryRenderTexture);

	PostInitCallback& postInitMulticast = GEngine->GetPostInitMulticast();
	postInitMulticast.BindRaw(this, &ForwardRenderer::InitGI);
}


bool ForwardRenderer::TriangleTrace(const PathTracingRay& inRay, PathTracePayload& outPayload, glm::vec3& outColor)
{
	bool bHit = false;
	for (const RenderCommand& command : MainCommands)
	{
		if (command.Triangles.size() == 0)
		{
			continue;
		}

		PathTracePayload currMeshPayload;
		if (command.AccStructure.Trace(inRay, currMeshPayload))
		{
			bHit = true;
			if (currMeshPayload.Distance < outPayload.Distance)
			{
				outPayload = currMeshPayload;
				outColor = command.OverrideColor;
			}
		}
	}

	return bHit;
}

static eastl::vector<glm::vec4> lightCoeffs;
void ForwardRenderer::InitGI()
{
	SHSample* samples = new SHSample[SH_TOTAL_SAMPLE_COUNT];

	LOG_INFO("Initializing SH Samples");

	SphericalHarmonics::InitSamples(samples);

	LOG_INFO("Building BVH");

	int sceneCoeffCount = 0;
	for (RenderCommand& command : MainCommands)
	{

		// Precache transforms
		if (command.Triangles.size() == 0)
		{
			continue;
		}

		const eastl::shared_ptr<const DrawableObject> parent = command.Parent.lock();
		glm::mat4 model = parent->GetModelMatrix();

		if (!command.AccStructure.IsValid())
		{
			eastl::vector<PathTraceTriangle> transformedTriangles = command.Triangles;
			//for (PathTraceTriangle& triangle : transformedTriangles)
			//{
			//	triangle.Transform(model);
			//}
			command.AccStructure.Build(transformedTriangles);
		}

		// Each vertex has its own SH Probe and SH_COEFFICIENT_COUNT coefficients
		command.TransferCoeffs.resize(command.Vertices.size() * SH_COEFFICIENT_COUNT);
		for (glm::vec3& coeff : command.TransferCoeffs)
		{
			coeff = glm::vec3(0.f, 0.f, 0.f);
		}
	}

#ifdef _DEBUG
	LOG_INFO("Tracing.. This is a lot faster in Release");
#else
	LOG_INFO("Tracing..");
#endif // _DEBUG


	for (RenderCommand& command : MainCommands)
	{
		PathTracingRay traceRay;
		PathTracePayload payload;
		glm::vec3 color;

		command.CoeffsBuffer = RHI::Get()->CreateTextureBuffer(command.Vertices.size() * SH_COEFFICIENT_COUNT * sizeof(glm::vec3));

		for (int32_t v = 0; v < command.Vertices.size(); ++v)
		{
			const Vertex& vert = command.Vertices[v];
			// For each vertex, evaluate all samples of its SH Sphere

			int32_t nrTraces = 0;
			for (int s = 0; s < SH_TOTAL_SAMPLE_COUNT; s++)
			{
				float dot = glm::dot(vert.Normal, samples[s].Direction);
				// Proceed only with samples within the hemisphere defined by the Vertex Normal
				// all other samples will be 0
				if (dot >= 0.0f)
				{
					++nrTraces;

					traceRay.Origin = vert.Position + (vert.Normal * 0.001f);
					traceRay.Direction = samples[s].Direction;

					const bool hit = TriangleTrace(traceRay, payload, color);

					// If the Ray was not occluded
					if (!hit)
					{
						// For diffuse materials, compose the transfer vector.
						// This vector includes the BDRF, incorporating the albedo colour, a lambertian diffuse factor (dot) and a SH sample
						for (int i = 0; i < SH_COEFFICIENT_COUNT; i++)
						{
							// Add the contribution of this sample
							//command.TransferCoeffs[v * SH_COEFFICIENT_COUNT + i] += command.OverrideColor * dot * samples[s].Coeffs[i];
							command.TransferCoeffs[v * SH_COEFFICIENT_COUNT + i] += glm::vec3(1.f, 1.f, 1.f) * samples[s].Coeffs[i];
						}
						//command.TransferCoeffs[v * SH_COEFFICIENT_COUNT] += glm::vec3(1.f, 1.f, 1.f);
					}
				}
			}
			//command.TransferCoeffs[v * SH_COEFFICIENT_COUNT] /= float(nrTraces * 10);


			// Probability to sample any point on the surface of the unit sphere is the same for all samples,
			// meaning that the weighting function is 1/surface area of unit sphere which is 4*PI => probability function p(x) is 1/(4*PI).
			// => The constant weighting function which we need to multiply our Coeffs by is 1 / p(x) = 4 * PI

			// Monte Carlo sampling means that we need to normalize all coefficients by N
			// => normalization factor of (4 * PI) / N multiplied with the Sum of samples.

			const float normalization_factor = 4.0f * PI / SH_TOTAL_SAMPLE_COUNT;

			//Normalize coefficients
			for (int i = 0; i < SH_COEFFICIENT_COUNT; i++)
			{
				command.TransferCoeffs[v * SH_COEFFICIENT_COUNT + i] *= normalization_factor;
			}
		}

		ASSERT(command.TransferCoeffs.size() == command.Vertices.size() * SH_COEFFICIENT_COUNT);
		const size_t finalSize = command.TransferCoeffs.size() * sizeof(glm::vec3);
		RHI::Get()->UploadDataToBuffer(*command.CoeffsBuffer, &command.TransferCoeffs[0], finalSize);
	}


	// Light Coefficients
	{
		lightCoeffs.resize(SH_COEFFICIENT_COUNT);
		// For each sample
		for (int s = 0; s < SH_TOTAL_SAMPLE_COUNT; s++)
		{
			const float theta = samples[s].Theta;
			const float phi = samples[s].Phi;

			// For each SH coefficient
			for (int n = 0; n < SH_COEFFICIENT_COUNT; n++)
			{
				// The reason this works is kind of a happy mistake. Normally, theta would be the angle, starting from the top but because the formulas
				// to get cartesian from spherical here is based on a coordinate base that has Z as up, theta here is based on Z which points towards the screen
				// thus illuminating like a light coming from Z to -Z(because that's how SH are added)
				// Also, theta and phi are inversed compared to the usual mathematical notation
				const glm::vec3 sampleValue = theta < PI / 6.f ? glm::vec3(1.f, 1.f, 1.f) : glm::vec3(0.f, 0.f, 0.f);
				const glm::vec3 res = sampleValue * samples[s].Coeffs[n];

				lightCoeffs[n] += glm::vec4(res.x, res.y, res.z, 1.f);
			}
		}

		// Weighed by the area of a 3D unit sphere
		const float weight = 4.0f * PI;
		// Divide the result by weight and number of samples
		const float factor = weight / SH_TOTAL_SAMPLE_COUNT;

		for (int i = 0; i < SH_COEFFICIENT_COUNT; i++)
		{
			lightCoeffs[i] *= factor;
		}
	}


}

static bool bBVHDebugDraw = false;
void ForwardRenderer::DisplaySettings()
{
	ImGui::Checkbox("BVH Debug Draw", &bBVHDebugDraw);

	static bool bOverrideColor = true;
	//ImGui::Checkbox("Override Color", &bOverrideColor);

	UniformsCache["bOverrideColor"] = bOverrideColor;
}

void ForwardRenderer::Draw()
{
	ImGui::Begin("Renderer settings");

	DisplaySettings();

	SetBaseUniforms();
	UpdateUniforms();

	SetLightingConstants();

	// Clear default framebuffer buffers
	RHI::Get()->ClearBuffers();

	RHI::Instance->BindDefaultFrameBuffer();

	// Clear additional framebuffer buffers
	RHI::Instance->ClearBuffers();



    DrawCommands(MainCommands);

	// Draw debug primitives
	DrawDebugManager::Draw();

	SetDrawMode(EDrawMode::Default);

	ImGui::End();
}

void ForwardRenderer::Present()
{
	RHI::Get()->SwapBuffers();
}

void ForwardRenderer::SetLightingConstants()
{
	const eastl::vector<eastl::shared_ptr<LightSource>>& lights = SceneManager::Get().GetCurrentScene().GetLights();
	
	eastl::vector<eastl::shared_ptr<LightSource>> dirLights;
	eastl::vector<eastl::shared_ptr<LightSource>> pointLights;

	for (const eastl::shared_ptr<LightSource>& light : lights)
	{
		switch (light->LData.Type)
		{
		case ELightType::Directional:
		{
			dirLights.push_back(light);
			break;
		}
		case ELightType::Point:
		{
			pointLights.push_back(light);
			break;
		}
		}
	}

	ASSERT(dirLights.size() <= 1);

	//////////////////////////////////////////////////////////////////////////
	// ImGui
	//////////////////////////////////////////////////////////////////////////

	const glm::vec3 cameraPos = SceneManager::Get().GetCurrentScene().GetCurrentCamera()->GetAbsoluteTransform().Translation;

	constexpr glm::vec3 forward = glm::vec3(0.f, 0.f, 1.f);
	const glm::vec3 cameraForward = SceneManager::Get().GetCurrentScene().GetCurrentCamera()->GetAbsoluteTransform().Rotation * forward;

	const bool useDirLight = dirLights.size() > 0;
	UniformsCache["bUseDirLight"] = (int32_t)useDirLight;

	if (useDirLight)
	{
		const eastl::shared_ptr<LightSource>& dirLight = dirLights[0];

		const glm::vec3 dir = dirLight->GetAbsoluteTransform().Rotation * glm::vec3(0.f, 0.f, 1.f);
		UniformsCache["DirectionalLightDirection"] = glm::normalize(dir);
	}
	else
	{
		UniformsCache["DirectionalLightDirection"] = glm::vec3(0.f, 0.f, 0.f);
	}


	eastl::vector<SPointLight> shaderPointLightData;
	for (const eastl::shared_ptr<LightSource>& light : pointLights)
	{
		SPointLight pointLight;

		const Transform& lightTransf = light->GetAbsoluteTransform();
		pointLight.position = glm::vec4(lightTransf.Translation.x, lightTransf.Translation.y, lightTransf.Translation.z, 0.f);

		const PointLightData& pointData = light->LData.TypeData.PointData;
		
		pointLight.linear = pointData.Linear;
		pointLight.quadratic = pointData.Quadratic;
		pointLight.color = glm::vec4(light->LData.Color.x, light->LData.Color.y, light->LData.Color.z, 0.f);

		shaderPointLightData.push_back(pointLight);
	}

	UniformsCache["NumPointLights"] =  static_cast<int32_t>(shaderPointLightData.size());
	UniformsCache["PointLights"] = shaderPointLightData;

	ASSERT(lights.size() > 0);

	glm::vec3 lightRot = lights[0]->GetRelRotation();

	const glm::quat rotation = glm::quat(glm::radians(lightRot));
	eastl::vector<glm::vec4> rotatedLightCoeffs;
	SphericalHarmonicsRotation::Rotate(rotation, lightCoeffs, rotatedLightCoeffs);
	UniformsCache["LightCoeffs"] = rotatedLightCoeffs;
}

glm::mat4 CreateMyOrthoLH(float left, float right, float bottom, float top, float zNear, float zFar)
{
	glm::mat4 Result(1);
	Result[0][0] = 2.f / (right - left);
	Result[1][1] = 2.f / (top - bottom);
	Result[2][2] = 1.f / (zFar - zNear);
	Result[3][0] = (left + right) / (left - right);
	Result[3][1] = (top + bottom) / (bottom - top);
	Result[3][2] = zNear / (zNear - zFar);

	return Result;
}

void ForwardRenderer::UpdateUniforms()
{
	const glm::mat4 view = SceneManager::Get().GetCurrentScene().GetCurrentCamera()->GetLookAt();
	UniformsCache["view"] = view;
}

void ForwardRenderer::DrawCommands(const eastl::vector<RenderCommand>& inCommands)
{
	// TODO: Fix the draw mode thing with objects being able to tell which passes they are in
	// and the renderer being able to use certain passes
	if (CurrentDrawMode & EDrawMode::Default)
	{
		const int32_t builtInPassesCount = glm::log2((int)EDrawMode::Count);
		for (int i = 0; i < builtInPassesCount; ++i)
		{
			const EDrawMode::Type currentMode = static_cast<EDrawMode::Type>(1 << i);
			SetDrawMode(currentMode);

			for (const RenderCommand& renderCommand : inCommands)
			{
				if (renderCommand.DrawPasses & currentMode)
				{
					DrawCommand(renderCommand);
				}
			}
		}
	}
	else
	{
		for (const RenderCommand& renderCommand : inCommands)
		{
			if (CurrentDrawMode & EDrawMode::DEPTH && !renderCommand.Material->bCastShadow)
			{
				continue;
			}

			DrawCommand(renderCommand);
		}
	}
}

void ForwardRenderer::DrawCommand(const RenderCommand& inCommand)
{
	const bool parentValid = !inCommand.Parent.expired();
	if (!ENSURE(parentValid))
	{
		return;
	}

	const eastl::shared_ptr<const DrawableObject> parent = inCommand.Parent.lock();
	const eastl::shared_ptr<RenderMaterial> material = GetMaterial(inCommand);
	const eastl::shared_ptr<MeshDataContainer>& dataContainer = inCommand.DataContainer;

	if (!parent->IsVisible() || !material)
	{
		return;
	}

	RHI::Get()->BindVertexBuffer(*(dataContainer->VBuffer));

	// Additional vertex data buffers
  	for (const eastl::shared_ptr<RHIVertexBuffer>& additionalBuffer : dataContainer->AdditionalBuffers)
  	{
  		constexpr bool bindIndexBuffer = false;
  		RHI::Get()->BindVertexBuffer(*(additionalBuffer), bindIndexBuffer);
  	}

	RHI::Get()->BindShader(*(material->Shader));

	material->ResetUniforms();

	UniformsCache["model"] = parent->GetModelMatrix();
	UniformsCache["ObjPos"] = parent->GetAbsoluteTransform().Translation;
	UniformsCache["OverrideColor"] = inCommand.OverrideColor;


	// Path Tracing Debug

	if (inCommand.Triangles.size() != 0)
	{
		static bool bDrawTrianglesDebug = false;
		ImGui::Checkbox("Triangles Centers Debug Draw", &bDrawTrianglesDebug);
		if (bDrawTrianglesDebug)
		{
			const float InvTriangleCount = inCommand.Triangles.size();
			glm::vec3 center = glm::vec3(0.f, 0.f, 0.f);
			for (const PathTraceTriangle& triangle : inCommand.Triangles)
			{
				glm::vec3 triangleCenter = (triangle.V[0] + triangle.V[1] + triangle.V[2]) * 0.3333333333333333333333f;

				DrawDebugHelpers::DrawDebugPoint(triangleCenter, 0.03f, glm::vec3(1.f, 0.f, 0.f));

				center += triangleCenter * InvTriangleCount;
			}

			DrawDebugHelpers::DrawDebugPoint(center, 0.1f);
		}

		if (bBVHDebugDraw)
		{
			RenderCommand& nonConstCommand = const_cast<RenderCommand&>(inCommand);
			if (!nonConstCommand.AccStructure.IsValid())
			{
				//eastl::vector<PathTraceTriangle> transformedTriangles = nonConstCommand.Triangles;
				//for (PathTraceTriangle& triangle : transformedTriangles)
				//{
				//	triangle.Transform(model);
				//}

				nonConstCommand.AccStructure.Build(nonConstCommand.Triangles);
			}
			nonConstCommand.AccStructure.Root->DebugDraw();
		}
	}


// Path Tracing Debug


// Pathtrace

RHI::Get()->BindTextureBuffer(*inCommand.CoeffsBuffer, 0);

// Pathtrace

	//{
	//	int texNr = 0;
	//	for (const eastl::shared_ptr<RHITexture2D>& tex : material->OwnedTextures)
	//	{
	//		RHI::Get()->BindTexture2D(*tex, texNr);
	//		++texNr;
	//	}

	//	for (const eastl::weak_ptr<RHITexture2D>& tex : material->ExternalTextures)
	//	{
	//		if (tex.expired())
	//		{
	//			continue;
	//		}

	//		eastl::shared_ptr<RHITexture2D>& sharedTex = tex.lock();

	//		RHI::Get()->BindTexture2D(*sharedTex, texNr);
	//		++texNr;
	//	}
	//}

	const uint32_t indicesCount = dataContainer->VBuffer->GetIndicesCount();

	parent->UpdateCustomUniforms(UniformsCache);
	material->SetUniformsValue(UniformsCache);
	material->BindBuffers();

	switch (inCommand.DrawType)
	{
	case EDrawType::DrawElements:
	{
		RHI::Get()->DrawElements(indicesCount);

		break;
	}
	case EDrawType::DrawArrays:
	{
		//glDrawArrays(GL_TRIANGLES, 0, indicesCount);
		break;
	}
	case EDrawType::DrawInstanced:
	{
		RHI::Get()->DrawInstanced(indicesCount, inCommand.InstancesCount);
		break;
	}
	}

	// Additional vertex data buffers
//  	for (const eastl::shared_ptr<RHIVertexBuffer>& additionalBuffer : dataContainer->AdditionalBuffers)
//  	{
//  		constexpr bool unbindIndexBuffer = false;
//  		RHI::Instance->UnbindVertexBuffer(*(additionalBuffer), unbindIndexBuffer);
//  	}

	RHI::Get()->UnbindVertexBuffer(*(dataContainer->VBuffer));

	// Pathtrace

	RHI::Get()->UnbindTextureBuffer(*inCommand.CoeffsBuffer, 0);

	// Pathtrace

	//{
	//	int texNr = 0;
	//	for (const eastl::shared_ptr<RHITexture2D>& tex : material->OwnedTextures)
	//	{
	//		RHI::Get()->UnbindTexture2D(*tex, texNr);
	//		++texNr;
	//	}

	//	for (const eastl::weak_ptr<RHITexture2D>& tex : material->ExternalTextures)
	//	{
	//		if (tex.expired())
	//		{
	//			continue;
	//		}

	//		eastl::shared_ptr<RHITexture2D>& sharedTex = tex.lock();
	//		RHI::Get()->BindTexture2D(*sharedTex, texNr);
	//		++texNr;
	//	}
	//}

	material->UnbindBuffers();
	RHI::Get()->UnbindShader(*(material->Shader));
}

eastl::shared_ptr<RenderMaterial> ForwardRenderer::GetMaterial(const RenderCommand& inCommand) const
{
	switch (CurrentDrawMode)
	{
	case EDrawMode::Default:
	{
		return inCommand.Material;
	}
	}

	return { nullptr };
}

void ForwardRenderer::SetVSyncEnabled(const bool inEnabled)
{
	//glfwSwapInterval(inEnabled);
}

void ForwardRenderer::AddCommand(const RenderCommand & inCommand)
{
	MainCommands.push_back(inCommand);
}

void ForwardRenderer::AddDecalCommand(const RenderCommand& inCommand)
{}

void ForwardRenderer::AddCommands(eastl::vector<RenderCommand> inCommands)
{
#ifdef _DEBUG
	for (RenderCommand& command : inCommands)
	{
		ASSERT(command.DataContainer);
		ASSERT(command.Material);
	}
#endif

	MainCommands.insert(MainCommands.end(), inCommands.begin(), inCommands.end());
}

void ForwardRenderer::SetDrawMode(const EDrawMode::Type inDrawMode)
{
	CurrentDrawMode = inDrawMode;
}

bool ForwardRenderer::GetOrCreateContainer(const eastl::string& inInstanceName, OUT eastl::shared_ptr<MeshDataContainer>& outContainer)
{
	ASSERT(!inInstanceName.empty());

	using iterator = const eastl::unordered_map<eastl::string, eastl::shared_ptr<MeshDataContainer>>::iterator;
	const iterator& containerIter = RenderDataContainerMap.find(inInstanceName);
	const bool materialExists = containerIter != RenderDataContainerMap.end();

	if (materialExists)
	{
		outContainer = (*containerIter).second;

		return true;
	}

	eastl::shared_ptr<MeshDataContainer> newContainer = eastl::make_shared<MeshDataContainer>();
	RenderDataContainerMap[inInstanceName] = newContainer;
	outContainer = newContainer;

	return false;
}

eastl::string ForwardRenderer::GetMaterialsDirPrefix()
{
	return "Forward";
}

void ForwardRenderer::SetViewportSizeToMain()
{
	const WindowsWindow& currentWindow = GEngine->GetMainWindow();
	const WindowProperties& props = currentWindow.GetProperties();
	RHI::Get()->SetViewportSize(props.Width, props.Height);
}

