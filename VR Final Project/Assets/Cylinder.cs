using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Cylinder : MonoBehaviour {
    public int _band;
    public float _startScale, _scaleMultiplier;
	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        transform.localScale = new Vector3(transform.localScale.x, (Instrument.bands[_band] * _scaleMultiplier) + _startScale, transform.localScale.z);
	}
}
