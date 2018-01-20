void sendfloat(float value) {
  byte* p;
  p = (byte*)&value;
  for (int i = 0; i < 4; i++)
    Serial3.print((char) * (p + i));
}
