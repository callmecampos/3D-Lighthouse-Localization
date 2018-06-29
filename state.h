enum State {
  PreSync, Sync,
  PreVert, PreHoriz,
  Vert, Horiz,
  Waiting, Hit
};

void start();
void finish();
