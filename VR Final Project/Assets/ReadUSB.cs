using UnityEngine;
using System.IO.Ports;
using System.Collections.Generic;

public class ReadUSB : MonoBehaviour {

    const int baudrate = 115200;

	const string portName = "/dev/tty.usbmodem2822491";
    //const string portName = "COM4";
	SerialPort serialPort;

	void getPortNames ()
	{
		int p = (int)System.Environment.OSVersion.Platform;
		List<string> serial_ports = new List<string> ();

		// Are we on Unix?
		if (p == 4 || p == 128 || p == 6) {
			string[] ttys = System.IO.Directory.GetFiles ("/dev/", "tty.*");
			foreach (string dev in ttys) {
				if (dev.StartsWith ("/dev/tty.*"))
					serial_ports.Add (dev);
				Debug.Log (System.String.Format (dev));
			}
		}
	}

    void Start () {
		getPortNames ();
		serialPort = new SerialPort(portName, baudrate);
        serialPort.ReadTimeout = 2000;
        serialPort.Open();

        if( !serialPort.IsOpen ) {

                Debug.LogError("Couldn't open " + portName);

        }
    }

	void set(Quaternion q) {
//		Vector3 axis = new Vector3 (0, 0, 0);
//		float angle = 0.0f;
//		q.ToAngleAxis(out angle, out axis);
//		transform.localPosition.Set(0,0,1);
//		transform.localRotation.Set (0, 0, 0, 0);
//		transform.RotateAround(transform.parent.position, axis, angle);
		transform.rotation=q;
	}


    void Update () {

            string buffer = serialPort.ReadExisting();

            string[] lines = buffer.Split( '\n' );

            if ( lines.Length >= 2) {

                    string[] line = lines[lines.Length - 2].Split( ' ' );

			if (line [0] == "QC") {

				Quaternion q = new Quaternion (float.Parse (line [2]),
					                                       float.Parse (line [3]),
					                                       -float.Parse (line [4]),
					                                       float.Parse (line [1]));

				set(Quaternion.Inverse (q));

			} else if (line [0] == "QLM") {
				Quaternion q = new Quaternion (float.Parse (line [2]),
					                                       float.Parse (line [3]),
					                                       -float.Parse (line [4]),
					                                       float.Parse (line [1]));
				set(Quaternion.Inverse (q));
				// Vector3 pos = new Vector3(float.Parse(line[8]), float.Parse(line[9]), float.Parse(line[10]));
				Vector3 pos = new Vector3 (0.01f * float.Parse (line [8]), 1 + 0.01f * float.Parse (line [9]), 1.0f);
				transform.position = pos;
			} else if (line [0] == "QHM") {
				Quaternion q = new Quaternion (float.Parse (line [2]),
					float.Parse (line [3]),
					-float.Parse (line [4]),
					float.Parse (line [1]));
				set(Quaternion.Inverse (q));
				Vector3 pos = new Vector3 (-0.01f * float.Parse (line [5]), 1 + 0.01f * float.Parse (line [6]), 1.0f);
				transform.position = pos;

			}


            }

    }


    void OnGUI()
    {
            string euler = "Euler angle: " + transform.eulerAngles.x + ", " +
                           transform.eulerAngles.y + ", " + transform.eulerAngles.z;

            GUI.Label(new Rect(10,10,500,100), euler);
    }
}
