int bytesToUint16(int[] arr, int from) {
  return arr[from] + arr[from + 1] * 256;
}

float bytesToFloat(int[] arr, int from) {
  int buf = 0;
  buf |= (arr[from] & 0xFF);
  buf |= (arr[from + 1] & 0xFF) << 8;
  buf |= (arr[from + 2] & 0xFF) << 16;
  buf |= (arr[from + 3] & 0xFF) << 24;
  return Float.intBitsToFloat(buf);
}
