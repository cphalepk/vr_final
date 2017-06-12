using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class Instrument : MonoBehaviour {

	AudioSource instrument;
    public static float[] samples = new float[512];
    public static float[] bands = new float[8];
    // CharacterController controller;
    // Use this for initialization
    Vector3 planePosition = new Vector3(0,0,5);
	void Start () {
		instrument = GetComponent<AudioSource> ();
	}

	Vector2 doPlayingPlaneProjection(Transform transform) {
		Quaternion y_axis = new Quaternion (0, 1, 0, 0);
		//Quaternion invRotation = new Quaternion (transform.rotation.x, transform.rotation.y, transform.rotation.z, -transform.rotation.w);
		//Quaternion rotatedY = new Quaternion (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w).premultiply (y_axis).premultiply (invRotation);

//		Quaternion intermediate = y_axis * transform.rotation;
//		Quaternion rotatedY = invRotation * intermediate;
		Quaternion rotatedY = transform.rotation * y_axis * Quaternion.Inverse(transform.rotation);
		Vector3 wandDirection = new Vector3 (rotatedY.x, rotatedY.y, rotatedY.z);

		float s = (planePosition.z - transform.position.z) / wandDirection.z;
		if (s < 0) {
			return new Vector2 (leftmax,upmax);
		}
		Vector3 proj = transform.position + s * wandDirection;
		if (proj.z < 0) {

		}
		return new Vector2 (proj.x, proj.y);
	}

	const float upmax = 10.0f;
	const float downmax = 1.0f;
	const float leftmax = -5.0f;
	const float rightmax = 5.0f;
	const float pitchmin = 0.75f;
	const float pitchmax = 3.0f;

	void doModulation (Vector2 point) {

		//Do ranging
		if (float.IsNaN (point.y) || point.y > upmax) {
			point.y = upmax;
		}
		if (point.y < downmax) {
			point.y = downmax;
		}
		if (float.IsNaN (point.x) || point.x < leftmax) {
			point.x = leftmax;
		}
		if (point.x > rightmax) {
			point.x = rightmax;
		}
		//Debug.Log (point.ToString ());
		instrument.volume = (float)( (20*Math.Exp(upmax - point.y) - 1.0)/(Math.Exp(upmax) - 1.0));
		//instrument.pitch = (float)( ((pitchmax - pitchmin)*Math.Exp(rightmax - leftmax - (-point.x - leftmax)) - 1.0)/(Math.Exp(rightmax - leftmax) - 1.0) + pitchmin);
		instrument.pitch = (float) (pitchmin + (pitchmax - pitchmin) * (rightmax - leftmax - (-point.x - leftmax))/(rightmax - leftmax));
	}

    void MakeBands()
    {
        int count = 0;
        for (int i = 0; i < 8; i++)
        {
            float average = 0.0f;
            int sampleCount = (int)Mathf.Pow(2, i + 1);
            if (i == 7) sampleCount += 2;
            for (int j = 0; j < sampleCount; j++)
            {
                average += samples[count] * (count + 1);
                count++;
            }
            average /= count;
            bands[i] = average;
        }
    }

	// Update is called once per frame
	void Update () {
		Transform transform = instrument.transform.parent;
		Vector2 proj = doPlayingPlaneProjection (transform);
		if (Input.GetKey ("space")) {
			doModulation (proj);
		}
        instrument.GetSpectrumData(samples, 0, FFTWindow.Blackman);
        MakeBands();
	}
}
