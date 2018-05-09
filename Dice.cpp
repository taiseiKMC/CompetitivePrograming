
enum Face
{
	Front,Up,Back,Down,Left,Right,
};
template<typename T>
struct Dice
{
	T pip[6];
	T& operator[] (const Face id)
	{
		return pip[id];
	}
	const T& operator[] (const Face id) const
	{
		return pip[id];
	}

	void rotate(Face f1, Face f2, Face f3, Face f4)
	{
		int tmp=pip[f1];
		pip[f1]=pip[f2];
		pip[f2]=pip[f3];
		pip[f3]=pip[f4];
		pip[f4]=tmp;
	}
	void rollx() { rotate(Up, Front, Down, Back); }
	void rollxi() { rotate(Up, Back, Down, Front); }
	void rolly() { rotate(Up, Left, Down, Right); }
	void rollyi() { rotate(Up, Right, Down, Left); }
	void rollz() { rotate(Back, Left, Front, Right); }
	void rollzi() { rotate(Back, Right, Front, Left); }
};