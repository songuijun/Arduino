int x = -160;

void setup() 
{
  size(300, 100);
  fill(125, 0, 250);
  textSize(50);
}

void draw() 
{
  background(255); 
  text("HELLOW", x, 50); 
  x++;                
  if(x == width){
    x = -160;      
  }
}

