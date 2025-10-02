void SetRelays(void)
{
 if (!useMCP23017) return;  // ✅ Früh abbrechen wenn MCP nicht verfügbar
    
  if (bitRead(relay, 0)) mcp.digitalWrite(8, HIGH);  else mcp.digitalWrite(8, LOW);
  if (bitRead(relay, 1)) mcp.digitalWrite(9, HIGH);  else mcp.digitalWrite(9, LOW);
  if (bitRead(relay, 2)) mcp.digitalWrite(10, HIGH); else mcp.digitalWrite(10, LOW);
  if (bitRead(relay, 3)) mcp.digitalWrite(11, HIGH); else mcp.digitalWrite(11, LOW);
  if (bitRead(relay, 4)) mcp.digitalWrite(12, HIGH); else mcp.digitalWrite(12, LOW);
  if (bitRead(relay, 5)) mcp.digitalWrite(13, HIGH); else mcp.digitalWrite(13, LOW);
  if (bitRead(relay, 6)) mcp.digitalWrite(14, HIGH); else mcp.digitalWrite(14, LOW);
  if (bitRead(relay, 7)) mcp.digitalWrite(15, HIGH); else mcp.digitalWrite(15, LOW);

   //if (bitRead(relay, 0)) Serial.println("0 high");

}
