using System.Collections;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Text;
using TMPro;
using UnityEngine;

public class UdpSend : MonoBehaviour
{
    public string _message;
    public TextMeshProUGUI Sending_Text;

    private string host;
    private int port;
    private UdpClient client;

    // Start is called before the first frame update
    void Start()
    {
        host = "127.0.0.1";
        port = 64276;
        client = new UdpClient();
        client.Connect(host, port);
    }

    // Update is called once per frame
    void Update()
    {
        var message = Encoding.UTF8.GetBytes(_message);
        Sending_Text.text = _message;
        client.Send(message, message.Length);
    }

    public void OnDestroy()
    {
        client.Close();
        client.Dispose();
        client = null;
    }
}
