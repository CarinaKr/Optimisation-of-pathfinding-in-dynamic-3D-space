// based on: https://answers.unity.com/questions/1123326/jsonutility-array-not-supported.html
using System;
using System.Collections;
using System.Collections.Generic;
using Pathfinding.Nodes;
using UnityEngine;

namespace Util
{
    public static class JsonHelper
    {
        public static string ArrayToJson<T>(T[] array)
        {
            Wrapper<T> wrapper = new Wrapper<T> ();
            wrapper.array = array;
            return JsonUtility.ToJson (wrapper);
        }

        public static T[] JsonToArray<T>(string json)
        {
            Wrapper<T> wrapper = JsonUtility.FromJson<Wrapper<T>>(json);
            return wrapper.array;
        }
 
        [Serializable]
        private class Wrapper<T>
        {
            public T[] array;
        }
    }
}
