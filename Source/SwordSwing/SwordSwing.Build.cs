// Fill out your copyright notice in the Description page of Project Settings.

using UnrealBuildTool;

public class SwordSwing : ModuleRules
{
	public SwordSwing(TargetInfo Target)
	{
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "ProceduralMeshComponent", "Synthesis",  "UMG", "Slate", "SlateCore"});
        PublicIncludePathModuleNames.AddRange(new string[] { "Synthesis" });
        PrivateDependencyModuleNames.AddRange(new string[] {  });
        PrivateIncludePathModuleNames.AddRange(new string[] {  });
        PublicAdditionalLibraries.Add(@"C:/Program Files (x86)/Epic Games/projects/SwordSwing/ThirdParty/voro++/libs/voro++.lib");

        PublicIncludePaths.Add(@"C:/Program Files (x86)/Epic Games/projects/SwordSwing/ThirdParty/voro++/includes");
        // Uncomment if you are using Slate UI
        // PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

        // Uncomment if you are using online features
        // PrivateDependencyModuleNames.Add("OnlineSubsystem");

        // To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
    }
}
