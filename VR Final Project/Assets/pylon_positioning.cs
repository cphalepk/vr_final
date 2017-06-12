using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class pylon_positioning : MonoBehaviour {

    Quaternion initialRotation;

	// Use this for initialization
	void Start () {
        initialRotation = transform.rotation;
	}
	
	// Update is called once per frame
	void Update () {
        //transform.rotation = initialRotation;
        Transform parent = transform.parent;
        transform.rotation = parent.rotation * initialRotation * Quaternion.Inverse(parent.rotation) ;
    }
}
