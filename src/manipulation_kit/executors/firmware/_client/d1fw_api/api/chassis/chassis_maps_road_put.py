from http import HTTPStatus
from typing import Any
from urllib.parse import quote

import httpx

from ... import errors
from ...client import AuthenticatedClient, Client
from ...models.chassis_maps_road_put_response_200 import ChassisMapsRoadPutResponse200
from ...models.error_envelope import ErrorEnvelope
from ...models.road_put import RoadPut
from ...types import Response


def _get_kwargs(
    scene: str,
    *,
    body: RoadPut,
) -> dict[str, Any]:
    headers: dict[str, Any] = {}

    _kwargs: dict[str, Any] = {
        "method": "put",
        "url": "/v1/chassis/maps/{scene}/road".format(
            scene=quote(str(scene), safe=""),
        ),
    }

    _kwargs["json"] = body.to_dict()

    headers["Content-Type"] = "application/json"

    _kwargs["headers"] = headers
    return _kwargs


def _parse_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> ChassisMapsRoadPutResponse200 | ErrorEnvelope | None:
    if response.status_code == 200:
        response_200 = ChassisMapsRoadPutResponse200.from_dict(response.json())

        return response_200

    if response.status_code == 400:
        response_400 = ErrorEnvelope.from_dict(response.json())

        return response_400

    if response.status_code == 404:
        response_404 = ErrorEnvelope.from_dict(response.json())

        return response_404

    if response.status_code == 409:
        response_409 = ErrorEnvelope.from_dict(response.json())

        return response_409

    if response.status_code == 502:
        response_502 = ErrorEnvelope.from_dict(response.json())

        return response_502

    if response.status_code == 504:
        response_504 = ErrorEnvelope.from_dict(response.json())

        return response_504

    if client.raise_on_unexpected_status:
        raise errors.UnexpectedStatus(response.status_code, response.content)
    else:
        return None


def _build_response(
    *, client: AuthenticatedClient | Client, response: httpx.Response
) -> Response[ChassisMapsRoadPutResponse200 | ErrorEnvelope]:
    return Response(
        status_code=HTTPStatus(response.status_code),
        content=response.content,
        headers=response.headers,
        parsed=_parse_response(client=client, response=response),
    )


def sync_detailed(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: RoadPut,
) -> Response[ChassisMapsRoadPutResponse200 | ErrorEnvelope]:
    """Replace one map's road network

     Whole-sub-resource replacement, and therefore idempotent: the two lists are the scene's complete
    road network afterwards, not a delta. They are forwarded to the base as opaque JSON, because the
    base's own description of this write declares them as bare lists with no element type. Gated by the
    chassis soft-kill latch, because the road network is where the base may drive. `data` is the base's
    own reply, forwarded unchanged.

    Args:
        scene (str):
        body (RoadPut): `PUT /v1/chassis/maps/{scene}/road`.

            Whole-resource replacement: the two lists are the scene's complete road
            network afterwards, not a delta. Both are forwarded to the mobile base as
            opaque JSON, because the base's own description of its `saveMapRoad` body
            declares them as bare lists with no element type, so this daemon does not
            invent a shape the base does not attest.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsRoadPutResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
        body=body,
    )

    response = client.get_httpx_client().request(
        **kwargs,
    )

    return _build_response(client=client, response=response)


def sync(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: RoadPut,
) -> ChassisMapsRoadPutResponse200 | ErrorEnvelope | None:
    """Replace one map's road network

     Whole-sub-resource replacement, and therefore idempotent: the two lists are the scene's complete
    road network afterwards, not a delta. They are forwarded to the base as opaque JSON, because the
    base's own description of this write declares them as bare lists with no element type. Gated by the
    chassis soft-kill latch, because the road network is where the base may drive. `data` is the base's
    own reply, forwarded unchanged.

    Args:
        scene (str):
        body (RoadPut): `PUT /v1/chassis/maps/{scene}/road`.

            Whole-resource replacement: the two lists are the scene's complete road
            network afterwards, not a delta. Both are forwarded to the mobile base as
            opaque JSON, because the base's own description of its `saveMapRoad` body
            declares them as bare lists with no element type, so this daemon does not
            invent a shape the base does not attest.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsRoadPutResponse200 | ErrorEnvelope
    """

    return sync_detailed(
        scene=scene,
        client=client,
        body=body,
    ).parsed


async def asyncio_detailed(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: RoadPut,
) -> Response[ChassisMapsRoadPutResponse200 | ErrorEnvelope]:
    """Replace one map's road network

     Whole-sub-resource replacement, and therefore idempotent: the two lists are the scene's complete
    road network afterwards, not a delta. They are forwarded to the base as opaque JSON, because the
    base's own description of this write declares them as bare lists with no element type. Gated by the
    chassis soft-kill latch, because the road network is where the base may drive. `data` is the base's
    own reply, forwarded unchanged.

    Args:
        scene (str):
        body (RoadPut): `PUT /v1/chassis/maps/{scene}/road`.

            Whole-resource replacement: the two lists are the scene's complete road
            network afterwards, not a delta. Both are forwarded to the mobile base as
            opaque JSON, because the base's own description of its `saveMapRoad` body
            declares them as bare lists with no element type, so this daemon does not
            invent a shape the base does not attest.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        Response[ChassisMapsRoadPutResponse200 | ErrorEnvelope]
    """

    kwargs = _get_kwargs(
        scene=scene,
        body=body,
    )

    response = await client.get_async_httpx_client().request(**kwargs)

    return _build_response(client=client, response=response)


async def asyncio(
    scene: str,
    *,
    client: AuthenticatedClient | Client,
    body: RoadPut,
) -> ChassisMapsRoadPutResponse200 | ErrorEnvelope | None:
    """Replace one map's road network

     Whole-sub-resource replacement, and therefore idempotent: the two lists are the scene's complete
    road network afterwards, not a delta. They are forwarded to the base as opaque JSON, because the
    base's own description of this write declares them as bare lists with no element type. Gated by the
    chassis soft-kill latch, because the road network is where the base may drive. `data` is the base's
    own reply, forwarded unchanged.

    Args:
        scene (str):
        body (RoadPut): `PUT /v1/chassis/maps/{scene}/road`.

            Whole-resource replacement: the two lists are the scene's complete road
            network afterwards, not a delta. Both are forwarded to the mobile base as
            opaque JSON, because the base's own description of its `saveMapRoad` body
            declares them as bare lists with no element type, so this daemon does not
            invent a shape the base does not attest.

    Raises:
        errors.UnexpectedStatus: If the server returns an undocumented status code and Client.raise_on_unexpected_status is True.
        httpx.TimeoutException: If the request takes longer than Client.timeout.

    Returns:
        ChassisMapsRoadPutResponse200 | ErrorEnvelope
    """

    return (
        await asyncio_detailed(
            scene=scene,
            client=client,
            body=body,
        )
    ).parsed
