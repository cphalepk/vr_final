using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpotLightController : MonoBehaviour {

    // Use this for initialization
    Light spotlight;
    Vector3 planePosition = new Vector3(0, 0, 5);
    const float leftmax = -3.0f;
    const float rightmax = 3.0f;
    const float huemin = 0.0f;
    const float huemax = 0.85f;

    void Start () {
        spotlight = GetComponent<Light> ();
	}

    float doPlayingPlaneProjection(Transform transform)
    {
        Quaternion y_axis = new Quaternion(0, 1, 0, 0);
        //Quaternion invRotation = new Quaternion (transform.rotation.x, transform.rotation.y, transform.rotation.z, -transform.rotation.w);
        //Quaternion rotatedY = new Quaternion (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w).premultiply (y_axis).premultiply (invRotation);

        //		Quaternion intermediate = y_axis * transform.rotation;
        //		Quaternion rotatedY = invRotation * intermediate;
        Quaternion rotatedY = transform.rotation * y_axis * Quaternion.Inverse(transform.rotation);
        Vector3 wandDirection = new Vector3(rotatedY.x, rotatedY.y, rotatedY.z);

        float s = (planePosition.z - transform.position.z) / wandDirection.z;
        if (s < 0)
        {
            return leftmax;
        }
        Vector3 proj = transform.position + s * wandDirection;
        if (proj.z < 0)
        {

        }
        return proj.x;
    }

    void changeColor(float xpos)
    {
        if (float.IsNaN(xpos) || xpos < leftmax)
        {
            xpos = leftmax;
        }
        if (xpos > rightmax)
        {
            xpos = rightmax;
        }
        float hue = (float)(huemin + (huemax - huemin) * (rightmax - leftmax - (-xpos - leftmax)) / (rightmax - leftmax));
        spotlight.color = Color.HSVToRGB(hue,1.0f,1.0f);
    }

    // Update is called once per frame
    void Update () {
        Transform parentTransform = spotlight.transform.parent;
        float xpos = doPlayingPlaneProjection(parentTransform);
        changeColor(xpos);
    }
}
