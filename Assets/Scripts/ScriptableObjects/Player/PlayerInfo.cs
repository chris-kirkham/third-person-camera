using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[CreateAssetMenu(fileName = "PlayerInfo", menuName = "ScriptableObjects/sPlayerInfo")]
public class PlayerInfo : ScriptableObject
{
    public GameObject player;

    public Vector3 Position { get; private set; } = Vector3.zero;
    public float Speed { get; private set; } = 0f;
    public float HorizontalSpeed { get; private set; } = 0f; //speed of player discounting y axis velocity
}
