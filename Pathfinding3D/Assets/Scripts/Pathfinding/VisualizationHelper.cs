using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Pathfinding
{
    [CreateAssetMenu]
    public class VisualizationHelper : ScriptableObject
    {
        [SerializeField] private Color[] expandedColors;

        public Color ExpandedColor => expandedColors[expandedColorIndex];

        private int expandedColorIndex = 0;

        public void SetNextExpandedColor()
        {
            expandedColorIndex = (expandedColorIndex + 1) % expandedColors.Length;
        }
    }
}
