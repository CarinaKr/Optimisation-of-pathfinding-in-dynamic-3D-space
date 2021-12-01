//this class is part of the Async Await Support package by Modest Tree Media
//source: https://assetstore.unity.com/packages/tools/integration/async-await-support-101056

using System.Runtime.CompilerServices;
using System.Threading.Tasks;

public class WaitForBackgroundThread
{
    public ConfiguredTaskAwaitable.ConfiguredTaskAwaiter GetAwaiter()
    {
        return Task.Run(() => { }).ConfigureAwait(false).GetAwaiter();
    }
}