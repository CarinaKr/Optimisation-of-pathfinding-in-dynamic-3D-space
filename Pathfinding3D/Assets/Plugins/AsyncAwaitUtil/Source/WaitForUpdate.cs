//this class is part of the Async Await Support package by Modest Tree Media
//source: https://assetstore.unity.com/packages/tools/integration/async-await-support-101056

using UnityEngine;

// This can be used as a way to return to the main unity thread when using multiple threads
// with async methods
public class WaitForUpdate : CustomYieldInstruction
{
    public override bool keepWaiting => false;
}