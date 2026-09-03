#version 330 core

out vec4 FragColor;

uniform float u_Level;
uniform vec3 u_Background;
uniform float u_FogMin;
uniform float u_FogMax;
uniform sampler2D u_Heightmap;
uniform sampler2D u_Normalmap;
uniform sampler2D u_Texture;
uniform vec3 u_CameraPos;

in vec3 Color;
in vec3 Normal;
in vec3 FragPos;
in vec2 TexCoord;
flat in float Factor;

vec3 calculateDirLight(vec3 direction, vec3 normal, vec3 color)
{
	float ka = 0.6;
	float kd = 1.0f;

	vec3 light_color = vec3(1.0);

  // ambient
  vec3 ambient = ka * light_color;
  
  // diffuse 
  vec3 diffuse = max(dot(normalize(normal), normalize(direction)), 0.0) * light_color;

  return (ambient + diffuse) * color;
}

void main()
{
  vec3 lightDir = vec3(-2.0, 4.0, -1.0);

  // distance fog towards the sky color
  vec4 fogColor = vec4(u_Background, 1.0);

  float dist = length(FragPos.xyz - u_CameraPos);
  float fogFactor = (u_FogMax - dist) / (u_FogMax - u_FogMin);
  fogFactor = clamp(fogFactor, 0.0, 1.0);

  // negative mipmap bias sharpens the terrain texture (less trilinear blur)
  vec4 terrainColor = vec4(calculateDirLight(lightDir, Normal, texture(u_Texture, TexCoord, -0.75).rgb), 1.0);
  FragColor = mix(fogColor, terrainColor, fogFactor);
}
