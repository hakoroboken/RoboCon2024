using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ButtonScript : MonoBehaviour
{
    public bool _sendcan = true;

    public GameObject _touch;
    public GameObject _OK;
    public GameObject _STOP;

    private Image touch;
    private Image OK;
    private Image STOP;

    public Sprite[] _touchimage;
    public Sprite[] _OKimage;
    public Sprite[] _STOPimage;

    public void OKfun(){
        _sendcan = true;
        touch.sprite = _touchimage[0];
        OK.sprite = _OKimage[1];
        STOP.sprite = _STOPimage[0];
    }

    public void STOPfun(){
        _sendcan = false;
        touch.sprite = _touchimage[1];
        OK.sprite = _OKimage[0];
        STOP.sprite = _STOPimage[1];
    }
    
    // Start is called before the first frame update
    void Start()
    {
        touch = _touch.GetComponent<Image>();
        OK = _OK.GetComponent<Image>();
        STOP = _STOP.GetComponent<Image>();
    }
}
