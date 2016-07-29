using UnityEngine;
using System;
using System.Collections.Generic;

namespace Summer
{
	class EffectFootprint : SummerBehaviour
	{
		public int _maxFootPrints;
		public Vector2 _size;
		public float _depth_offset;
		public float _alpha_decal;

		public Material _material = null;

		private LinkedList<Vector3> _vertList = new LinkedList<Vector3>();
		private LinkedList<Vector2> _uvList = new LinkedList<Vector2>();
		private LinkedList<int> _triList = new LinkedList<int>();
		private LinkedList<Color> _colorList = new LinkedList<Color>();

		private Mesh _mesh = null;
		private MeshFilter _meshFilter = null;
		private MeshRenderer _renderer = null;

		private int _ver_per_face = 4;
		private int _tri_per_face = 6;
		private int _count = 0;
		private Vector3[] _face_vert;
		private bool _fp_number_changed = false;

		public void init()
		{
			_meshFilter = (MeshFilter)gameObject.AddComponent<MeshFilter>();
			_renderer = (MeshRenderer)gameObject.AddComponent<MeshRenderer>();

			_mesh = new Mesh();

			if (null != _material)
				_renderer.material = _material;
			else
				my.error("Miss material for the foot print!!");

			_meshFilter.mesh = _mesh;

			_face_vert = new Vector3[4];
			_face_vert[0] = new Vector3(-_size.x * 0.5f, _depth_offset, -_size.y * 0.5f);
			_face_vert[1] = new Vector3(-_size.x * 0.5f, _depth_offset, _size.y * 0.5f);
			_face_vert[2] = new Vector3(_size.x * 0.5f, _depth_offset, _size.y * 0.5f);
			_face_vert[3] = new Vector3(_size.x * 0.5f, _depth_offset, -_size.y * 0.5f);

		}

		void Update()
		{
			if (0 == _vertList.Count)
				return;

			if (_count > _maxFootPrints)
				_removeFootPrint(_count - _maxFootPrints);

			if (!_fp_number_changed)
			{
				//fade all the face
				LinkedListNode<Color> node = _colorList.First;
				while (node != null)
				{
					Color col = node.Value;
					col.a -= _alpha_decal;
					if (col.a < 0.0f)
						node.Value = Color.clear;
					else
						node.Value = col;

					node = node.Next;
				}

				Color[] cols = new Color[_colorList.Count];
				_colorList.CopyTo(cols, 0);
				_mesh.colors = cols;
			}
			else
			{
				_fp_number_changed = false;
				uploadMesh();
			}
		}

		private void uploadMesh()
		{
			Vector3[] vs = new Vector3[_vertList.Count];
			Vector2[] uvs = new Vector2[_uvList.Count];
			int[] tris = new int[_triList.Count];
			Color[] cols = new Color[_colorList.Count];

			_vertList.CopyTo(vs, 0);
			_uvList.CopyTo(uvs, 0);
			_triList.CopyTo(tris, 0);
			_colorList.CopyTo(cols, 0);

			_mesh.Clear();
			_mesh.vertices = vs;
			_mesh.triangles = tris;
			_mesh.uv = uvs;
			_mesh.colors = cols;
		}

		private void _removeFootPrint(int delCount)
		{
			for (int i = 0; i < _ver_per_face * delCount; ++i)
			{
				_vertList.RemoveFirst();
				_uvList.RemoveFirst();
				_colorList.RemoveFirst();
			}

			for (int i = 0; i < _tri_per_face * delCount; ++i)
				_triList.RemoveFirst();

			//since the count has changed, we need to update all the idx
			LinkedListNode<int> node = _triList.First;
			while (node != null)
			{
				node.Value -= _ver_per_face;
				node = node.Next;
			}

			_count -= delCount;
			_fp_number_changed = true;
		}

		public void AddFootprint(bool isLeft, Matrix4x4 matrix)
		{
			_fp_number_changed = true;

			++_count;

			for (int i = 0; i < 4; ++i)
			{
				Vector3 vect = matrix.MultiplyPoint(_face_vert[i]);
				_vertList.AddLast(vect);
				_colorList.AddLast(Color.white);
			}

			if (isLeft)
			{
				_uvList.AddLast(new Vector2(1.0f, 1.0f));
				_uvList.AddLast(new Vector2(1.0f, 0.0f));
				_uvList.AddLast(new Vector2(0.0f, 0.0f));
				_uvList.AddLast(new Vector2(0.0f, 1.0f));
			}
			else
			{
				_uvList.AddLast(new Vector2(1.0f, 0.0f));
				_uvList.AddLast(new Vector2(1.0f, 1.0f));
				_uvList.AddLast(new Vector2(0.0f, 1.0f));
				_uvList.AddLast(new Vector2(0.0f, 0.0f));
			}

			int count = _vertList.Count;
			_triList.AddLast(count - 4);
			_triList.AddLast(count - 3);
			_triList.AddLast(count - 2);
			_triList.AddLast(count - 4);
			_triList.AddLast(count - 2);
			_triList.AddLast(count - 1);
		}
	}
}
