class MeshObject(object):

    def __init__(self, desc_str):
        self.instance_id = None
        self.category = None
        self.aligned_dims = None
        self.aligned_planes = None

        self.parse_desc_(desc_str)


    def __str__(self):
        ret = ""

        ret += "Mesh Object:\n"

        ret += " - Instance ID: {}\n".format(self.instance_id)
        ret += " - Category: {}\n".format(self.category)
        ret += " - Aligned dimension: {}\n".format(self.aligned_dims)
        ret += " - Aligned planes: {}".format(['\n', "None"][len(self.aligned_planes) == 0])
        for plane in self.aligned_planes:
            ret += "    * {}\n".format(plane)

        return ret


    def parse_desc_(self, desc_str):
        """
        CAD Model Description Line

        <instance-id>,<category>,<aligned-dimension>,<aligned-plane>

        <aligned-dimension>: <x>\,<y>\,<z>
        <aligned-plane>: <a>\,<b>\,<c>\,<d>\.<a>\,<b>\,<c>\,<d> ...
        """
        # remove the followed '\n' at the end of the line
        # remove char `"`
        # replace `\,` with whitespace ` `, use whitespace as delimiter for <x, y, z> and <a, b, c, d>
        # replace `\.` with comma `,`, use comma as delimiter for aligned planes
        desc_str = desc_str.strip('\n').replace('"', '').replace("\,", ' ').replace("\.", ',')
        tokens = desc_str.split(',')

        self.instance_id = tokens[0]
        self.category = tokens[1]
        self.aligned_dims = tuple([float(i) for i in tokens[2].split(' ')])
        self.aligned_planes = []

        for tkn in tokens[3:]:
            if tkn == '':
                break
            self.aligned_planes.append(
                tuple([float(i) for i in tkn.split(' ')])
            )
        