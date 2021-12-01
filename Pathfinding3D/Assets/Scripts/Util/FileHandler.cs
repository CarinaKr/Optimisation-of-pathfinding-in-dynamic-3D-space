using System.IO;
using UnityEngine;

namespace Util
{
    public class FileHandler : MonoBehaviour
    {
        public static FileHandler self;
        [SerializeField] private string debugFilePath;
        [SerializeField] private string saveFilePath;

        private StreamWriter debugWriter;
        private StreamWriter saveFileWriter;
        private StreamReader saveFileReader;

        private void Awake()
        {
            if (self == null)
            {
                self = this;
            }

            if (self != this)
            {
                Destroy(gameObject);
            }
        }

        private void OnEnable()
        {
            debugWriter = new StreamWriter(debugFilePath, true);
        }

        private void OnDisable()
        {
            debugWriter.Close();
        }

        public void WriteDebugString(string message)
        {
            debugWriter.Write(message);
        }

        public void WriteSaveFile(string message)
        {
            if (saveFileWriter == null)
            {
                saveFileWriter = new StreamWriter(saveFilePath, false);
            }
            saveFileWriter.Write(message);
            saveFileWriter.Flush();
            saveFileWriter.Close();
        }

        public void ReadSaveFile(out string savedNodes, out string predecessors)
        {
            if (saveFileReader == null)
            {
                saveFileReader = new StreamReader(saveFilePath);
            }
            savedNodes = saveFileReader.ReadLine();
            predecessors = saveFileReader.ReadLine();
            saveFileReader.Close();
        }
    }
}