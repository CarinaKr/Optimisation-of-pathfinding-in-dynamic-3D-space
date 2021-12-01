//this class is part of the Async Await Support package by Modest Tree Media
//source: https://assetstore.unity.com/packages/tools/integration/async-await-support-101056

using System;
using UnityEngine;

// TODO: Remove the allocs here, use a static memory pool?
public static class Awaiters
{
    public static WaitForUpdate NextFrame { get; } = new WaitForUpdate();

    public static WaitForFixedUpdate FixedUpdate { get; } = new WaitForFixedUpdate();

    public static WaitForEndOfFrame EndOfFrame { get; } = new WaitForEndOfFrame();

    public static WaitForSeconds Seconds(float seconds)
    {
        return new WaitForSeconds(seconds);
    }

    public static WaitForSecondsRealtime SecondsRealtime(float seconds)
    {
        return new WaitForSecondsRealtime(seconds);
    }

    public static WaitUntil Until(Func<bool> predicate)
    {
        return new WaitUntil(predicate);
    }

    public static WaitWhile While(Func<bool> predicate)
    {
        return new WaitWhile(predicate);
    }
}