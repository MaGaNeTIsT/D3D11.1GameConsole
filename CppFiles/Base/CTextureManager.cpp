#include "../../Entry/MyMain.h"
#include "../../Headers/Base/CTextureManager.h"
#include "../../Headers/Base/CTexture2D.h"
#include "../../Headers/Base/CRenderDevice.h"

CTextureManager* CTextureManager::m_TextureManager = new CTextureManager();
void CTextureManager::Uninit()
{
	CTextureManager::ClearTexture2DData();
	delete m_TextureManager;
}
void CTextureManager::ClearTexture2DData()
{
	if (m_TextureManager->m_Texture2DData.size() > 0)
	{
		for (const auto& data : (m_TextureManager->m_Texture2DData))
		{
			if ((data.second) != NULL)
			{
				delete (data.second);
			}
		}
		m_TextureManager->m_Texture2DData.clear();
	}
}
CTexture2D* CTextureManager::LoadTexture2D(const std::string& name)
{
	const static std::string _tgaName = "tga";

	CTexture2D* resultTexture = CTextureManager::FindTexture2DData(name);
	if (resultTexture != NULL)
		return resultTexture;
	{
		size_t pos = name.find_last_of('.');
		if (pos == std::string::npos)
			return NULL;
		std::string typeName = name.substr(pos + 1, name.length());

		if (typeName == _tgaName)
			resultTexture = (CTextureManager::LoadTGATexture2D(name));
		else
			return NULL;
	}
	if (resultTexture == NULL)
		return NULL;
	CTextureManager::AddTexture2DData(name, resultTexture);
	return resultTexture;
}
CTexture2D* CTextureManager::LoadTGATexture2D(const std::string& name)
{
	TBYTE	header[18];
	TBYTE*	image;
	TBYTE	depth;
	UINT	width, height, bpp, size;
	FILE*	file;
	fopen_s(&file, name.c_str(), "rb");
	if (file == NULL)
		return NULL;
	fread_s(header, sizeof(header), sizeof(header), 1, file);
	width	= header[13] * 256 + header[12];
	height	= header[15] * 256 + header[14];
	depth	= header[16];
	if (depth == 32)
		bpp = 4;
	else if (depth == 24)
		bpp = 3;
	else
		bpp = 0;
	if (bpp != 4)
		return NULL;
	size	= width * height * bpp;
	image	= new TBYTE[size];
	fread_s(image, size, size, 1, file);
	// R<->B
	for (UINT y = 0; y < height; y++)
	{
		for (UINT x = 0; x < width; x++)
		{
			TBYTE c = image[(y * width + x) * bpp + 0];
			image[(y * width + x) * bpp + 0] = image[(y * width + x) * bpp + 2];
			image[(y * width + x) * bpp + 2] = c;
		}
	}
	fclose(file);
	Microsoft::WRL::ComPtr<ID3D11Texture2D> tempTexture2D = nullptr;
	{
		if (CRenderDevice::CreateTexture2D(tempTexture2D, width, height, DXGI_FORMAT_R8G8B8A8_UNORM, image, width * 4u, size) == false)
		{
			delete[]image;
			return NULL;
		}
	}
	Microsoft::WRL::ComPtr<ID3D11ShaderResourceView> tempSRV = nullptr;
	{
		if (CRenderDevice::CreateTexture2DShaderResourceView(tempTexture2D, tempSRV, DXGI_FORMAT_R8G8B8A8_UNORM, D3D11_SRV_DIMENSION_TEXTURE2D) == false)
		{
			delete[]image;
			return NULL;
		}
	}
	delete[]image;
	CTexture2D* resultTexture2D = new CTexture2D(name, tempTexture2D, tempSRV);
	return resultTexture2D;
}
void CTextureManager::AddTexture2DData(const std::string& name, CTexture2D* ptrData)
{
	m_TextureManager->m_Texture2DData[name] = ptrData;
}
CTexture2D* CTextureManager::FindTexture2DData(const std::string& name)
{
	std::map<std::string, CTexture2D*>::iterator element = m_TextureManager->m_Texture2DData.find(name);
	if (element == m_TextureManager->m_Texture2DData.end())
		return NULL;
	return (element->second);
}