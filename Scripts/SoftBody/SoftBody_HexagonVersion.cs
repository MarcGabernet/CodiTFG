using System.Collections;
using System.Collections.Generic;
using UnityEditor.PackageManager;
using UnityEngine;
using Unity.VisualScripting;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Assets.Scripts
{
    [RequireComponent(typeof(Collider))]
    [RequireComponent(typeof(Rigidbody))]
    [RequireComponent(typeof(SpringJoint))]
    public class SoftBody_HexagonVersion : MonoBehaviour
    {
        [Header("Dimensions dels subcomponents")]

        //Radi de les esferes subcomponent
        [SerializeField]
        private float radi = 0.2f;

        //Distancia entre subcomponents
        [SerializeField]
        private float distancia = 0.2f;

        [Header("Visual (PROVISIONAL)")]
        [SerializeField]
        private Material material;

        [SerializeField]
        private Material material2;

        [SerializeField]
        private Mesh componentEsfera;

        [SerializeField]
        private bool meshVisible;

        //Vertex que definiran on es creen els subcomponents
        private Vector3 vertexInferior;
        private Vector3 vertexSuperior;

        private ArrayList coordenadesSubcomponents;
        private Rigidbody rb;

        //Fills de l'objecte on guardarem tots els altres
        private GameObject subcomponents;
        private GameObject vertex;

        void ConvertirATou()
        {
            //Per odernar-ho creem els fills on guardarem els components
            subcomponents = new();
            subcomponents.name = "Subcomponents";
            subcomponents.transform.parent = transform;

            vertex = new();
            vertex.name = "Vertex";
            vertex.transform.parent = transform;

            //Declarem el rigidBody que farem servir pels subcomponents
            rb = GetComponent<Rigidbody>();
            rb.isKinematic = false;

            //Calcular el domini d'intent de generaci� de boles
            MeshFilter mf = rb.GetComponent<MeshFilter>();
            vertexInferior = CalculsDeVertex(mf.sharedMesh)[0];
            vertexSuperior = CalculsDeVertex(mf.sharedMesh)[1];

            //objecte que representar� cada subcomponent del sistema
            GameObject go = ObjecteAInstanciar(rb);

            //Assegurem que el valor de distancia no faci petar l'ordenador
            distancia = ComprovarValorsValids(distancia);

            //Fem els calculs
            coordenadesSubcomponents = new ArrayList();
            GenerarSubcomponents(vertexInferior, vertexSuperior, go);
            CrearConnexionsIAssignaPes(coordenadesSubcomponents);

            GetComponent<Collider>().enabled = false;

            CrearConnexioAlMesh();

            //Inhabilitem el components que no ens interessen de l'objecte inicial
            if (!meshVisible)
            {
                GetComponent<MeshRenderer>().enabled = false;
            }
            rb.isKinematic = true;
            DestroyImmediate(go);
        }

        Vector3[] CambiarEscalaAlsVertex(Vector3[] vertex)
        {
            for (int i = 0; i < vertex.Length; i++)
            {
                vertex[i] = Vector3.Scale(vertex[i], transform.localScale);
            }

            return vertex;
        }

        Vector3[] CalculsDeVertex(Mesh mesh)
        {
            Vector3[] VertexInferiorISuperior = new Vector3[2];

            Vector3[] vertexs = new Vector3[mesh.vertices.Length];
            vertexs = mesh.vertices;
            vertexs = CambiarEscalaAlsVertex(vertexs);

            float minX = vertexs[0].x; float maxX = vertexs[0].x;
            float minY = vertexs[0].y; float maxY = vertexs[0].y;
            float minZ = vertexs[0].z; float maxZ = vertexs[0].z;

            for (int i = 0; i < vertexs.Length; i++)
            {
                if (vertexs[i].x < minX)
                {
                    minX = vertexs[i].x;
                }
                if (vertexs[i].y < minY)
                {
                    minY = vertexs[i].y;
                }
                if (vertexs[i].z < minZ)
                {
                    minZ = vertexs[i].z;
                }
                if (vertexs[i].x > maxX)
                {
                    maxX = vertexs[i].x;
                }
                if (vertexs[i].y > maxY)
                {
                    maxY = vertexs[i].y;
                }
                if (vertexs[i].z > maxZ)
                {
                    maxZ = vertexs[i].z;
                }
            }

            VertexInferiorISuperior[0] = new Vector3(minX, minY, minZ);
            VertexInferiorISuperior[1] = new Vector3(maxX, maxY, maxZ);

            return VertexInferiorISuperior;
        }

        public GameObject ObjecteAInstanciar(Rigidbody rb)
        {
            GameObject obj = new();
            obj.layer = 3;

            //Rigidbody
            Rigidbody rb1 = obj.AddComponent<Rigidbody>();

            rb1.mass = rb.mass;
            rb1.drag = rb.drag;
            rb1.angularDrag = rb.angularDrag;
            rb1.useGravity = rb.useGravity;
            rb1.isKinematic = rb.isKinematic;
            rb1.interpolation = rb.interpolation;
            rb1.collisionDetectionMode = rb.collisionDetectionMode;
            rb1.constraints = rb.constraints;
            rb1.collisionDetectionMode = rb.collisionDetectionMode;

            obj.AddComponent<SphereCollider>();

            MeshFilter mf = obj.AddComponent<MeshFilter>();
            mf.mesh = componentEsfera;

            MeshRenderer mr = obj.AddComponent<MeshRenderer>();
            mr.material = material;

            return obj;
        }

        float ComprovarValorsValids(float dis)
        {
            if (dis < 0.01)
            {
                dis = 0.2f;
            }

            return dis;
        }

        void GenerarSubcomponents(Vector3 vertexInferior, Vector3 vertexSuperior, GameObject objecteInstanciat)
        {
            //En la generaci� de subcomponents la primera idea era que les eferes estiguessin contingudes estrictament dins l'objecte
            //(almenys dins el rectangle format pels valors extrems dels v�rtex) per� el fet que esferes que estan a la
            // frontera sobresurtin de l'objecte fa que la col�lisi� amb altres objectes sigui m�s bona (l'objecte no s'enfonsa tant) 

            bool filaXparella = true;
            bool filaYparella = true;

            Vector3 pos = new Vector3(vertexInferior.x /*+ radi / 2*/, vertexInferior.y /*+ radi / 2*/, vertexInferior.z /*+ radi / 2*/);

            int[] coords = new int[3];
            coords[0] = 0; //z
            coords[1] = 0; //x
            coords[2] = 0; //y

            float cos30 = Mathf.Sqrt(3) / 2f;

            while (pos.y <= vertexSuperior.y /*- radi / 2*/)
            {
                while (pos.x <= vertexSuperior.x /*- radi / 2*/)
                {
                    while (pos.z <= vertexSuperior.z /*- radi / 2*/)
                    {
                        ComprovaSiDinsObjecte(objecteInstanciat, pos, coords);

                        pos.z += distancia;
                        coords[0] += 1;
                    }
                    pos.z = vertexInferior.z /*+ radi / 2*/;
                    coords[0] = 0;

                    pos.x += distancia * cos30;
                    coords[1] += 1;

                    if (filaXparella)
                    {
                        pos.z += distancia / 2f;
                        filaXparella = false;
                    }
                    else
                    {
                        filaXparella = true;
                    }

                }
                pos.z = vertexInferior.z /*+ radi / 2*/;
                coords[0] = 0;
                pos.x = vertexInferior.x /*+ radi / 2*/;
                coords[1] = 0;

                pos.y += distancia * Mathf.Sqrt(2 / 3f);
                coords[2] += 1;

                if (filaYparella)
                {
                    pos.z += distancia / 2f;
                    pos.x += distancia / (4f * cos30);
                    filaYparella = false;
                    filaXparella = false;
                }
                else
                {
                    filaYparella = true;
                }
            }
            //canviem la posici� del fill "subcomponent" per a que les boles queden dins de l'objecte
            subcomponents.transform.position += transform.position;
        }

        void ComprovaSiDinsObjecte(GameObject objecteInstanciat, Vector3 pos, int[] coords)
        {
            //pos = transform.rotation * pos;

            pos += transform.position;

            Collider[] hitColliders = Physics.OverlapSphere(pos, 0f);

            if (true && hitColliders.Length > 0)
            {
                InstanciarSubcomponent(objecteInstanciat, pos, coords);
            }
        }

        void InstanciarSubcomponent(GameObject objecteInstanciat, Vector3 pos, int[] coords)
        {
            GameObject punt = Instantiate(objecteInstanciat, subcomponents.transform);
            punt.transform.position = pos - transform.position;
            punt.transform.localScale = Vector3.one * radi;

            punt.name = coords[0].ToString() + "_" + coords[1].ToString() + "_" + coords[2].ToString();
            coordenadesSubcomponents.Add(punt);
        }

        void CrearConnexionsIAssignaPes(ArrayList esferes)
        {
            int n = esferes.Count;

            /*float min = 2 * distancia - radi;
            if (min < 0)
            {
                min = 0;
            }//*/

            foreach (GameObject objecteSortida in esferes)
            { //ordre: z,x,y

                //Calcul de pes
                objecteSortida.GetComponent<Rigidbody>().mass = GetComponent<Rigidbody>().mass / (float)n;

                int[] coords = NomACoordenades(objecteSortida);

                /*Collider[] collidersAdjacents = Physics.OverlapSphere(objecteSortida.transform.position, min);
                Debug.Log(collidersAdjacents.Length);

                GameObject[] esferesAdjacents = new GameObject[collidersAdjacents.Length];

                for (int i = 0; i < collidersAdjacents.Length; i++)
                {
                    esferesAdjacents[i] = collidersAdjacents[i].gameObject;
                }//*/

                foreach (GameObject objecteArribada in esferes)
                {
                    //Crear connexions
                    ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 1, 0, 0));
                    ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 0, 1, 0));

                    if (coords[2] % 2 == 0)
                    {
                        if (coords[1] % 2 == 1)
                        {
                            ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 1, 1, 0));
                            ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 1, -1, 0));
                        }
                    }
                    else
                    {
                        ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 0, 0, 1));
                        ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 0, 0, -1));
                        ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 0, 1, 1));
                        ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 0, 1, -1));

                        if (coords[2] % 2 == 0)
                        {
                            ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 1, 0, 1));
                            ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, 1, 0, -1));
                        }
                        else
                        {
                            ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, -1, 0, 1));
                            ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, -1, 0, -1));
                            ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, -1, -1, 0));
                            ComprovaICreaConnexio(objecteSortida, objecteArribada, CoordenadesANom(coords, -1, 1, 0));
                        }
                    }
                }
            }
        }

        int[] NomACoordenades(GameObject obj)
        {
            int[] coordenades = new int[3];
            string[] coordenadesNom = obj.name.Split('_');

            for (int i = 0; i < 3; i++)
            {
                coordenades[i] = int.Parse(coordenadesNom[i]);
            }

            return coordenades;
        }

        string CoordenadesANom(int[] coords, int z, int x, int y)
        {
            return (coords[0] + z).ToString() + "_" + (coords[1] + x).ToString() + "_" + (coords[2] + y).ToString();
        }

        void ComprovaICreaConnexio(GameObject objecteSortida, GameObject objecteArribada, string nom)
        {
            if (objecteArribada.name == nom)
            {
                objecteSortida.AddComponent<SpringJoint>();
                SpringJoint[] joints = objecteSortida.GetComponents<SpringJoint>();
                SpringJoint sj = GetComponent<SpringJoint>();
                foreach (SpringJoint joint in joints)
                {
                    if (joint.connectedBody == null)
                    {
                        joint.connectedBody = objecteArribada.GetComponent<Rigidbody>();
                        joint.spring = sj.spring;
                        joint.damper = sj.damper;
                        joint.minDistance = sj.minDistance;
                        joint.maxDistance = sj.maxDistance;
                        joint.tolerance = sj.tolerance;
                        joint.breakForce = sj.breakForce;
                        joint.breakTorque = sj.breakTorque;
                        joint.enableCollision = sj.enableCollision;
                        joint.enablePreprocessing = sj.enablePreprocessing;
                        joint.massScale = sj.massScale;
                        joint.connectedMassScale = sj.connectedMassScale;
                    }
                }
            }
        }

        void CrearConnexioAlMesh()
        {
            MeshFilter mf = GetComponent<MeshFilter>();
            Mesh mesh = mf.sharedMesh;

            GameObject[] meshPoints = new GameObject[mesh.vertices.Length];

            for (int i = 0; i < mesh.vertices.Length; i++)
            {
                GameObject go = new();
                PropietatsMeshPoint(go, mesh);

                GameObject meshPoint = Instantiate(go, vertex.transform);
                meshPoint.transform.position = Vector3.Scale(mesh.vertices[i], transform.localScale);
                meshPoint.name = i.ToString();
                meshPoints[i] = meshPoint;

                DestroyImmediate(go);
            }

            vertex.transform.position += transform.position;

            float radiProva = 0.05f;
            int maxIntents = 50;

            foreach (GameObject go in meshPoints)
            {
                float increment = 0.05f;
                Collider[] hitColliders = Physics.OverlapSphere(go.transform.position, radiProva, LayerMask.NameToLayer("ESfera"));
                while (hitColliders.Length == 0 && radiProva / increment > maxIntents)
                {
                    radiProva += increment;
                    hitColliders = Physics.OverlapSphere(go.transform.position, radiProva);
                }
                if (hitColliders.Length == 0)
                {
                    Debug.LogError("S'ha fallat en adjuntar els v�rtex al objecte");
                    break;
                }
                FixedJoint fj = go.AddComponent<FixedJoint>();
                fj.connectedBody = BolaMesPropera(hitColliders, go).GetComponent<Rigidbody>();
            }
        }

        void PropietatsMeshPoint(GameObject go, Mesh mesh)
        {
            Rigidbody rb = go.AddComponent<Rigidbody>();
            rb.useGravity = false;
            rb.isKinematic = false;
            rb.mass = 0;
            go.transform.localScale = Vector3.one * 0.05f;

            MeshFilter mf = go.AddComponent<MeshFilter>();
            mf.mesh = mesh;

            MeshRenderer mr = go.AddComponent<MeshRenderer>();
            mr.material = material2;
            go.transform.localScale = Vector3.one * 0.05f;
        }

        Collider BolaMesPropera(Collider[] objectesPropers, GameObject go)
        {
            Collider millorOpcio = null;
            float distaciaMesProximaAlQuadrat = Mathf.Infinity;
            Vector3 currentPosition = go.transform.position;
            foreach (Collider obj in objectesPropers)
            {
                Vector3 vectorFinsObjecte = obj.transform.position - currentPosition;
                float dSqrToTarget = vectorFinsObjecte.sqrMagnitude;
                if (dSqrToTarget < distaciaMesProximaAlQuadrat)
                {
                    distaciaMesProximaAlQuadrat = dSqrToTarget;
                    millorOpcio = obj;
                }
            }

            return millorOpcio;
        }


#if UNITY_EDITOR
        public static void ConvertirTou()
        {
            SoftBody_HexagonVersion objecteTou = Selection.activeGameObject.GetComponent<SoftBody_HexagonVersion>();
            objecteTou.ConvertirATou();
        }

        [MenuItem("Soft Body/ HEX: Convertir-Actualitzar Objecte")]
        public static void Actualitzar()
        {
            RevertirTou();
            ConvertirTou();
        }

        [MenuItem("Soft Body/ HEX: Eliminar Components Elastics")]
        public static void RevertirTou()
        {

            SoftBody_HexagonVersion objecteTou = Selection.activeGameObject.GetComponent<SoftBody_HexagonVersion>();
            objecteTou.gameObject.GetComponent<MeshRenderer>().enabled = true;
            objecteTou.gameObject.GetComponent<Collider>().enabled = true;
            while (objecteTou.transform.childCount > 0)
            {
                DestroyImmediate(objecteTou.transform.GetChild(0).gameObject);
            }
        }

#endif

        Vector3[] ActualitzaMesh(Vector3[] vertices)
        {
            for (int i = 0; i < vertices.Length; i++)
            {
                vertices[i] = vertex.transform.GetChild(i).position;
                vertices[i] = Quaternion.Inverse(transform.rotation) * (vertices[i] - vertex.transform.position) + vertex.transform.position;
                vertices[i] -= transform.position;
                vertices[i].x /= transform.localScale.x;
                vertices[i].y /= transform.localScale.y;
                vertices[i].z /= transform.localScale.z;
            }

            return vertices;
        }

        Mesh mesh;
        Vector3[] vertices;

        void Start()
        {
            vertex = transform.GetChild(1).gameObject;
            mesh = GetComponent<MeshFilter>().mesh;
            vertices = mesh.vertices;
        }

        void Update()
        {
            mesh.vertices = ActualitzaMesh(vertices);
            mesh.RecalculateBounds();
        }

    }
}